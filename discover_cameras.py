from pypylon import pylon

def main():
    print("=" * 55)
    print("BASLER CAMERA DISCOVERY")
    print("=" * 55)

    factory = pylon.TlFactory.GetInstance()

    # Enumerate connected cameras
    devices = factory.EnumerateDevices()

    if not devices:
        print("\nNo Basler cameras detected.")
        return

    print(f"\nDetected {len(devices)} camera(s).\n")

    cameras = []

    for index, device in enumerate(devices):
        print(f"Camera {index + 1}")
        print("-" * 35)
        print(f"Model:          {device.GetModelName()}")
        print(f"Serial number:  {device.GetSerialNumber()}")
        print(f"Friendly name:  {device.GetFriendlyName()}")
        print(f"Device class:   {device.GetDeviceClass()}")
        print(f"Full name:      {device.GetFullName()}")

        # Create and open camera
        camera = pylon.InstantCamera(
            factory.CreateDevice(device)
        )

        try:
            camera.Open()

            print(f"Connected:      {camera.IsOpen()}")

            # Read camera configuration
            print(f"Width:          {camera.Width.GetValue()}")
            print(f"Height:         {camera.Height.GetValue()}")
            print(f"Pixel format:   {camera.PixelFormat.GetValue()}")

            if hasattr(camera, "AcquisitionFrameRate"):
                print(
                    f"Frame rate:     "
                    f"{camera.AcquisitionFrameRate.GetValue():.2f} FPS"
                )

            # Check available pixel formats
            print("\nPixel format options:")

            formats = camera.PixelFormat.GetSymbolics()

            for fmt in formats:
                print(f"  - {fmt}")

            cameras.append(camera)

        except Exception as error:
            print(f"Camera error: {error}")

        finally:
            if camera.IsOpen():
                camera.Close()

        print()

    print("=" * 55)
    print(f"DISCOVERY COMPLETE: {len(devices)} device(s) found")
    print("=" * 55)


if __name__ == "__main__":
    main()
