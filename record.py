import argparse
import csv
import json
import queue
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from pypylon import pylon


DEFAULT_FPS = 30.0
PIXEL_FORMAT = "BayerRG8"
DEFAULT_QUEUE_SIZE = 128
VIDEO_CODEC = "mp4v"


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Basler camera recorder"
    )
    parser.add_argument("duration", type=float)
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS)
    parser.add_argument("--left-serial")
    parser.add_argument("--right-serial")
    parser.add_argument("--queue-size", type=int, default=DEFAULT_QUEUE_SIZE)

    args = parser.parse_args()

    if args.duration <= 0 or args.fps <= 0 or args.queue_size <= 0:
        parser.error("Duration, FPS, and queue size must be positive.")

    if args.left_serial and args.left_serial == args.right_serial:
        parser.error("Camera serial numbers must differ.")

    return args


def discover_cameras(args):
    factory = pylon.TlFactory.GetInstance()
    devices = list(factory.EnumerateDevices())

    if not devices:
        raise RuntimeError("No Basler cameras detected.")
    if len(devices) > 2:
        raise RuntimeError("This recorder supports at most two cameras.")

    by_serial = {d.GetSerialNumber(): d for d in devices}

    requested = {
        s for s in (args.left_serial, args.right_serial)
        if s is not None
    }

    missing = requested - set(by_serial)
    if missing:
        raise RuntimeError(f"Camera serial numbers not found: {missing}")

    assigned = {}

    if args.left_serial:
        assigned["left"] = by_serial[args.left_serial]
    if args.right_serial:
        assigned["right"] = by_serial[args.right_serial]

    remaining = [
        d for d in devices
        if d.GetSerialNumber() not in requested
    ]
    remaining.sort(key=lambda d: d.GetSerialNumber())

    for device in remaining:
        if "left" not in assigned:
            assigned["left"] = device
        elif "right" not in assigned:
            assigned["right"] = device

    print(f"Detected {len(devices)} camera(s).")
    return assigned


def configure_camera(device, name, fps):
    factory = pylon.TlFactory.GetInstance()
    camera = pylon.InstantCamera(factory.CreateDevice(device))
    camera.Open()

    try:
        camera.PixelFormat.SetValue(PIXEL_FORMAT)
        camera.AcquisitionMode.SetValue("Continuous")

        if hasattr(camera, "AcquisitionFrameRateEnable"):
            node = camera.AcquisitionFrameRateEnable
            if node.IsWritable():
                node.SetValue(True)

        if not camera.AcquisitionFrameRate.IsWritable():
            raise RuntimeError(f"{name}: FPS setting is not writable.")

        camera.AcquisitionFrameRate.SetValue(fps)

        info = {
            "model": camera.GetDeviceInfo().GetModelName(),
            "serial": camera.GetDeviceInfo().GetSerialNumber(),
            "width": camera.Width.GetValue(),
            "height": camera.Height.GetValue(),
            "pixel_format": camera.PixelFormat.GetValue(),
            "target_fps": fps,
            "actual_fps": camera.AcquisitionFrameRate.GetValue(),
        }

        print(f"\n{name.upper()} CAMERA")
        for key, value in info.items():
            print(f"{key}: {value}")

        return camera, info

    except Exception:
        camera.Close()
        raise


def raw_writer_worker(name, output_dir, frame_queue, stop_event, stats):
    raw_path = output_dir / f"{name}.raw"
    csv_path = output_dir / f"{name}_timestamps.csv"
    frame_index = 0

    try:
        with open(raw_path, "wb", buffering=8 * 1024 * 1024) as raw_file:
            with open(csv_path, "w", newline="", buffering=1024 * 1024) as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow([
                    "frame_index",
                    "unix_timestamp_ns",
                    "monotonic_timestamp_ns",
                    "camera_timestamp_ticks",
                ])

                while True:
                    item = frame_queue.get()

                    if item is None:
                        break

                    frame, unix_ns, monotonic_ns, camera_ticks = item

                    raw_file.write(memoryview(frame).cast("B"))

                    writer.writerow([
                        frame_index,
                        unix_ns,
                        monotonic_ns,
                        camera_ticks,
                    ])

                    frame_index += 1
                    stats["written"] = frame_index

        stats["raw_bytes"] = (
            frame_index * stats["width"] * stats["height"]
        )

    except Exception as error:
        stats["errors"].append(f"Raw writer: {error}")
        stop_event.set()

    finally:
        print(f"{name}: saved {frame_index} raw frames.")


def acquisition_worker(
    name,
    camera,
    frame_queue,
    start_time,
    duration,
    stop_event,
    stats,
):
    deadline = start_time + duration
    deadline_ns = int(deadline * 1_000_000_000)
    start_ns = int(start_time * 1_000_000_000)

    print(f"{name}: acquisition started.")

    try:
        while not stop_event.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break

            timeout_ms = max(1, min(1000, int(remaining * 1000)))

            try:
                with camera.RetrieveResult(
                    timeout_ms,
                    pylon.TimeoutHandling_ThrowException,
                ) as result:

                    if not result.GrabSucceeded():
                        stats["grab_errors"] += 1
                        continue

                    monotonic_ns = time.monotonic_ns()

                    if monotonic_ns < start_ns:
                        stats["pre_start_frames"] += 1
                        continue

                    if monotonic_ns >= deadline_ns:
                        break

                    unix_ns = time.time_ns()
                    frame = np.ascontiguousarray(result.Array).copy()
                    camera_ticks = result.TimeStamp

                    stats["retrieved"] += 1

                    if stats["first_timestamp_ns"] is None:
                        stats["first_timestamp_ns"] = monotonic_ns

                    stats["last_timestamp_ns"] = monotonic_ns

                    try:
                        frame_queue.put_nowait((
                            frame,
                            unix_ns,
                            monotonic_ns,
                            camera_ticks,
                        ))
                        stats["queued"] += 1

                    except queue.Full:
                        stats["queue_dropped"] += 1

            except pylon.TimeoutException:
                stats["timeouts"] += 1

    except Exception as error:
        stats["errors"].append(f"Acquisition: {error}")
        stop_event.set()

    finally:
        print(f"{name}: retrieved {stats['retrieved']} frames.")


def generate_mp4(name, output_dir, info, stats):
    raw_path = output_dir / f"{name}.raw"
    video_path = output_dir / f"{name}.mp4"

    frame_count = stats["written"]
    if frame_count == 0:
        print(f"{name}: no frames available for MP4.")
        return

    width = info["width"]
    height = info["height"]
    fps = info["actual_fps"]

    expected_size = frame_count * width * height
    actual_size = raw_path.stat().st_size

    if actual_size != expected_size:
        raise RuntimeError(
            f"{name}: raw file size mismatch: "
            f"expected {expected_size}, got {actual_size}"
        )

    print(f"\n{name}: generating MP4 from {frame_count} frames.")

    frames = np.memmap(
        raw_path,
        dtype=np.uint8,
        mode="r",
        shape=(frame_count, height, width),
    )

    video_writer = cv2.VideoWriter(
        str(video_path),
        cv2.VideoWriter_fourcc(*VIDEO_CODEC),
        fps,
        (width, height),
        True,
    )

    if not video_writer.isOpened():
        del frames
        raise RuntimeError(f"Could not create {video_path}")

    try:
        for index in range(frame_count):
            bgr = cv2.cvtColor(
                frames[index],
                cv2.COLOR_BayerRG2BGR,
            )
            video_writer.write(bgr)

            if (index + 1) % 500 == 0:
                print(f"{name}: encoded {index + 1}/{frame_count}")

    finally:
        video_writer.release()
        del frames

    stats["video_frames"] = frame_count


def main():
    args = parse_arguments()

    print("=" * 55)
    print("BASLER CAMERA RECORDER")
    print("=" * 55)

    output_dir = (
        Path(__file__).resolve().parent
        / "videos"
        / datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f")
    )
    output_dir.mkdir(parents=True)

    print(f"Output: {output_dir}")

    assigned = discover_cameras(args)

    cameras = {}
    camera_info = {}
    frame_queues = {}
    stats = {}

    writer_threads = []
    acquisition_threads = []
    stop_event = threading.Event()
    shutdown_errors = []

    recording_start = None
    recording_end = None

    try:
        for name, device in assigned.items():
            camera, info = configure_camera(device, name, args.fps)

            cameras[name] = camera
            camera_info[name] = info

            frame_queues[name] = queue.Queue(
                maxsize=args.queue_size
            )

            stats[name] = {
                "retrieved": 0,
                "queued": 0,
                "written": 0,
                "queue_dropped": 0,
                "grab_errors": 0,
                "timeouts": 0,
                "pre_start_frames": 0,
                "raw_bytes": 0,
                "video_frames": 0,
                "first_timestamp_ns": None,
                "last_timestamp_ns": None,
                "errors": [],
                "width": info["width"],
                "height": info["height"],
            }

        # Start raw writer threads.
        for name in cameras:
            thread = threading.Thread(
                target=raw_writer_worker,
                args=(
                    name,
                    output_dir,
                    frame_queues[name],
                    stop_event,
                    stats[name],
                ),
                name=f"{name}_writer",
            )
            thread.start()
            writer_threads.append(thread)

        # Start all cameras.
        for camera in cameras.values():
            camera.StartGrabbing(pylon.GrabStrategy_OneByOne)

        print("\nAll cameras started.")

        recording_start = time.monotonic() + 1.0

        print(f"Duration: {args.duration}s")
        print(f"FPS: {args.fps}")
        print(f"Queue capacity: {args.queue_size} frames per camera")
        print("Press Ctrl+C to stop early.\n")

        for name, camera in cameras.items():
            thread = threading.Thread(
                target=acquisition_worker,
                args=(
                    name,
                    camera,
                    frame_queues[name],
                    recording_start,
                    args.duration,
                    stop_event,
                    stats[name],
                ),
                name=f"{name}_acquisition",
            )
            thread.start()
            acquisition_threads.append(thread)

        for thread in acquisition_threads:
            thread.join()

        recording_end = time.monotonic()

    except KeyboardInterrupt:
        print("\nRecording interrupted.")
        stop_event.set()
        recording_end = time.monotonic()

    except Exception:
        stop_event.set()
        recording_end = time.monotonic()
        raise

    finally:
        stop_event.set()

        for camera in cameras.values():
            try:
                if camera.IsGrabbing():
                    camera.StopGrabbing()
                if camera.IsOpen():
                    camera.Close()
            except Exception as error:
                shutdown_errors.append(str(error))

        # Drain queues before sending shutdown markers.
        for name, frame_queue in frame_queues.items():
            writer = next(
                (
                    t for t in writer_threads
                    if t.name == f"{name}_writer"
                ),
                None,
            )

            if writer is not None:
                while writer.is_alive():
                    try:
                        frame_queue.put(None, timeout=0.2)
                        break
                    except queue.Full:
                        continue

        for thread in writer_threads:
            thread.join()

    # Generate display videos only after acquisition has ended.
    video_errors = []

    for name in cameras:
        try:
            generate_mp4(
                name,
                output_dir,
                camera_info[name],
                stats[name],
            )
        except Exception as error:
            video_errors.append(f"{name}: {error}")

    actual_duration = None
    if recording_start is not None and recording_end is not None:
        actual_duration = recording_end - recording_start

    metadata = {
        "recording_timestamp": output_dir.name,
        "requested_duration_seconds": args.duration,
        "actual_recording_window_seconds": actual_duration,
        "requested_fps": args.fps,
        "pixel_format": PIXEL_FORMAT,
        "raw_format": "sequential uncompressed BayerRG8",
        "video_format": "MP4",
        "video_codec": VIDEO_CODEC,
        "video_encoding": "offline after acquisition",
        "synchronization": {
            "mode": "free-running",
            "hardware_trigger": False,
            "timestamps": "host retrieval timestamps",
        },
        "cameras": {},
        "errors": {
            "shutdown": shutdown_errors,
            "video": video_errors,
        },
    }

    for name, info in camera_info.items():
        metadata["cameras"][name] = {
            **info,
            "statistics": {
                key: value
                for key, value in stats[name].items()
                if key not in ("errors", "width", "height")
            },
            "errors": stats[name]["errors"],
        }

    with open(output_dir / "metadata.json", "w") as file:
        json.dump(metadata, file, indent=4)

    print("\n" + "=" * 55)
    print("RECORDING SUMMARY")
    print("=" * 55)

    for name, result in stats.items():
        print(f"\n{name.upper()}")
        print(f"Retrieved:    {result['retrieved']}")
        print(f"Queued:       {result['queued']}")
        print(f"Written:      {result['written']}")
        print(f"Dropped:      {result['queue_dropped']}")
        print(f"Grab errors:  {result['grab_errors']}")
        print(f"Timeouts:     {result['timeouts']}")
        print(f"Video frames: {result['video_frames']}")
        print(f"Raw size:     {result['raw_bytes'] / 1e9:.2f} GB")

    errors = (
        shutdown_errors
        + video_errors
        + [
            error
            for result in stats.values()
            for error in result["errors"]
        ]
    )

    if errors:
        print("\nCompleted with errors.")
        for error in errors:
            print(error)
    elif any(s["queue_dropped"] for s in stats.values()):
        print("\nCompleted with dropped frames.")
    else:
        print("\nCompleted successfully.")

    print(f"\nSaved to: {output_dir}")


if __name__ == "__main__":
    main()