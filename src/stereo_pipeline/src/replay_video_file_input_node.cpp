#include "stereo_pipeline/replay_video_file_input_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <cstring>
#include <utility>

#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;


ReplayVideoFileInputNode::ReplayVideoFileInputNode()
    : Node("replay_video_file_input_node")
{
    left_video_path_ = declare_parameter<std::string>(
        "left_video_path", "");

    right_video_path_ = declare_parameter<std::string>(
        "right_video_path", "");

    left_timestamps_path_ = declare_parameter<std::string>(
        "left_timestamps_path", "");

    right_timestamps_path_ = declare_parameter<std::string>(
        "right_timestamps_path", "");

    start_frame_ = declare_parameter<int>(
        "start_frame", 0);

    buffer_size_ = declare_parameter<int>(
        "buffer_size", 8);

    reliable_qos_ = declare_parameter<bool>(
        "reliable_qos", true);

    width_ = declare_parameter<int>(
        "width", 1920);

    height_ = declare_parameter<int>(
        "height", 1200);

    playback_fps_ = declare_parameter<double>(
        "playback_fps", 30.0);

    if (start_frame_ < 0 ||
        buffer_size_ <= 0 ||
        width_ <= 0 ||
        height_ <= 0 ||
        playback_fps_ <= 0.0)
    {
        throw std::invalid_argument(
            "Invalid playback parameters.");
    }

    if (left_video_path_.empty() ||
        right_video_path_.empty())
    {
        throw std::invalid_argument(
            "Both RAW file paths must be specified.");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    if (reliable_qos_) {
        qos.reliable();
    } else {
        qos.best_effort();
    }

    left_publisher_ =
        create_publisher<sensor_msgs::msg::Image>(
            "/stereo/left/image_raw", qos);

    right_publisher_ =
        create_publisher<sensor_msgs::msg::Image>(
            "/stereo/right/image_raw", qos);

    if (!initialize_videos()) {
        throw std::runtime_error(
            "RAW file initialization failed.");
    }

    if (!load_timestamps()) {
        throw std::runtime_error(
            "Timestamp initialization failed.");
    }

    stats_.decoder_start =
        std::chrono::steady_clock::now();

    decoder_thread_ = std::thread(
        &ReplayVideoFileInputNode::decoder_worker,
        this);

    const auto frame_period =
        std::chrono::duration<double>(
            1.0 / playback_fps_);

    playback_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            frame_period),
        std::bind(
            &ReplayVideoFileInputNode::publish_next_pair,
            this));

    RCLCPP_INFO(
        get_logger(),
        "========================================");

    RCLCPP_INFO(
        get_logger(),
        "RAW stereo replay initialized.");

    RCLCPP_INFO(
        get_logger(),
        "Resolution: %dx%d",
        width_,
        height_);

    RCLCPP_INFO(
        get_logger(),
        "Frames: %lu",
        static_cast<unsigned long>(
            frame_count_ - start_frame_));

    RCLCPP_INFO(
        get_logger(),
        "Playback FPS: %.2f",
        playback_fps_);

    RCLCPP_INFO(
        get_logger(),
        "Buffer capacity: %d",
        buffer_size_);

    RCLCPP_INFO(
        get_logger(),
        "========================================");
}


// ======================================================
// Destructor
// ======================================================


ReplayVideoFileInputNode::~ReplayVideoFileInputNode()
{
    stop_decoder();

    if (left_raw_file_.is_open()) {
        left_raw_file_.close();
    }

    if (right_raw_file_.is_open()) {
        right_raw_file_.close();
    }
}



// ======================================================
// Video initialization
// ======================================================

bool ReplayVideoFileInputNode::initialize_videos()
{
    // Open RAW files.
    left_raw_file_.open(
        left_video_path_,
        std::ios::binary
    );

    right_raw_file_.open(
        right_video_path_,
        std::ios::binary
    );

    if (!left_raw_file_.is_open() ||
        !right_raw_file_.is_open())
    {
        RCLCPP_ERROR(
            get_logger(),
            "Could not open both RAW files."
        );

        return false;
    }

    // Width and height were already declared
    // in the constructor. Do not declare them again.

    if (width_ <= 0 || height_ <= 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Invalid RAW image dimensions."
        );

        return false;
    }

    // BayerRG8: one byte per pixel.
    frame_size_ =
        static_cast<uint64_t>(width_) *
        static_cast<uint64_t>(height_);

    // Determine file sizes.
    left_raw_file_.seekg(0, std::ios::end);
    right_raw_file_.seekg(0, std::ios::end);

    const uint64_t left_file_size =
        static_cast<uint64_t>(left_raw_file_.tellg());

    const uint64_t right_file_size =
        static_cast<uint64_t>(right_raw_file_.tellg());

    // Every file must contain complete frames.
    if (left_file_size % frame_size_ != 0 ||
        right_file_size % frame_size_ != 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "RAW file size is not divisible by frame size."
        );

        return false;
    }

    const uint64_t left_frames =
        left_file_size / frame_size_;

    const uint64_t right_frames =
        right_file_size / frame_size_;

    frame_count_ = std::min(
        left_frames,
        right_frames
    );

    if (frame_count_ == 0 ||
        static_cast<uint64_t>(start_frame_) >= frame_count_)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Invalid frame count or starting frame."
        );

        return false;
    }

    // Seek to requested starting frame.
    const uint64_t start_offset =
        static_cast<uint64_t>(start_frame_) *
        frame_size_;

    left_raw_file_.clear();
    right_raw_file_.clear();

    left_raw_file_.seekg(
        static_cast<std::streamoff>(start_offset),
        std::ios::beg
    );

    right_raw_file_.seekg(
        static_cast<std::streamoff>(start_offset),
        std::ios::beg
    );

    if (!left_raw_file_.good() ||
        !right_raw_file_.good())
    {
        RCLCPP_ERROR(
            get_logger(),
            "Could not seek to starting frame."
        );

        return false;
    }

    RCLCPP_INFO(
        get_logger(),
        "RAW stereo sources opened."
    );

    RCLCPP_INFO(
        get_logger(),
        "Resolution: %dx%d",
        width_,
        height_
    );

    RCLCPP_INFO(
        get_logger(),
        "Frame size: %lu bytes",
        static_cast<unsigned long>(frame_size_)
    );

    RCLCPP_INFO(
        get_logger(),
        "Left frames: %lu",
        static_cast<unsigned long>(left_frames)
    );

    RCLCPP_INFO(
        get_logger(),
        "Right frames: %lu",
        static_cast<unsigned long>(right_frames)
    );

    RCLCPP_INFO(
        get_logger(),
        "Available stereo frames: %lu",
        static_cast<unsigned long>(frame_count_)
    );

    return true;
}


// ======================================================
// Timestamp CSV loading
// ======================================================

std::vector<ReplayVideoFileInputNode::FrameTimestamp>
ReplayVideoFileInputNode::read_timestamp_csv(
    const std::string & path)
{
    std::ifstream file(path);

    if (!file.is_open()) {
        throw std::runtime_error(
            "Cannot open timestamp CSV: " + path);
    }

    std::vector<FrameTimestamp> timestamps;

    std::string line;

    // Skip CSV header.
    std::getline(file, line);

    uint64_t expected_index = 0;

    while (std::getline(file, line)) {

        if (line.empty()) {
            continue;
        }

        std::stringstream stream(line);

        std::string frame_index_str;
        std::string unix_timestamp_str;
        std::string monotonic_timestamp_str;

        if (!std::getline(stream, frame_index_str, ',') ||
            !std::getline(stream, unix_timestamp_str, ',') ||
            !std::getline(stream, monotonic_timestamp_str, ','))
        {
            throw std::runtime_error(
                "Malformed timestamp CSV: " + path);
        }

        FrameTimestamp timestamp;

        const uint64_t frame_index =
            std::stoull(frame_index_str);

        timestamp.unix_ns =
            std::stoull(unix_timestamp_str);

        timestamp.monotonic_ns =
            std::stoull(monotonic_timestamp_str);

        if (frame_index != expected_index) {
            throw std::runtime_error(
                "Unexpected frame index in: " + path);
        }

        if (!timestamps.empty()) {

            if (timestamp.unix_ns <=
                    timestamps.back().unix_ns ||
                timestamp.monotonic_ns <=
                    timestamps.back().monotonic_ns)
            {
                throw std::runtime_error(
                    "Non-increasing timestamps in: " + path);
            }
        }

        timestamps.push_back(timestamp);

        ++expected_index;
    }

    return timestamps;
}


// ======================================================
// Load timestamp files
// ======================================================

bool ReplayVideoFileInputNode::load_timestamps()
{
    try {

        namespace fs = std::filesystem;

        if (left_timestamps_path_.empty()) {

            const fs::path video(left_video_path_);

            left_timestamps_path_ =
                (video.parent_path() /
                (video.stem().string() +
                "_timestamps.csv")).string();
        }

        if (right_timestamps_path_.empty()) {

            const fs::path video(right_video_path_);

            right_timestamps_path_ =
                (video.parent_path() /
                (video.stem().string() +
                "_timestamps.csv")).string();
        }

        left_timestamps_ =
            read_timestamp_csv(left_timestamps_path_);

        right_timestamps_ =
            read_timestamp_csv(right_timestamps_path_);

        frame_count_ = std::min({
            frame_count_,
            static_cast<uint64_t>(left_timestamps_.size()),
            static_cast<uint64_t>(right_timestamps_.size())
        });

        if (frame_count_ <=
            static_cast<uint64_t>(start_frame_))
        {
            throw std::runtime_error(
                "Insufficient timestamps for playback.");
        }

        RCLCPP_INFO(
            get_logger(),
            "Loaded %lu left and %lu right timestamps.",
            static_cast<unsigned long>(
                left_timestamps_.size()),
            static_cast<unsigned long>(
                right_timestamps_.size()));

        return true;
    }
    catch (const std::exception & error) {

        RCLCPP_ERROR(
            get_logger(),
            "Timestamp loading failed: %s",
            error.what());

        return false;
    }
}

// ======================================================
// Stop decoder
// ======================================================


void ReplayVideoFileInputNode::stop_decoder()
{
    stop_requested_.store(true);

    buffer_condition_.notify_all();

    if (decoder_thread_.joinable()) {
        decoder_thread_.join();
    }
}

// ======================================================
// Background decoder
// ======================================================

void ReplayVideoFileInputNode::decoder_worker()
{
    uint64_t decoded = 0;

    double total_decode_ms = 0.0;
    double max_decode_ms = 0.0;

    RCLCPP_INFO(
        get_logger(),
        "RAW decoder thread started.");

    try
    {
        for (uint64_t index = start_frame_;
             index < frame_count_ &&
             !stop_requested_.load();
             ++index)
        {
            const auto decode_start =
                std::chrono::steady_clock::now();

            StereoFramePair pair;

            pair.frame_index = index;

            // Preserve original timestamps.
            pair.left_timestamp =
                left_timestamps_.at(index);

            pair.right_timestamp =
                right_timestamps_.at(index);

            // Allocate BayerRG8 buffers.
            cv::Mat left_bayer(
                height_,
                width_,
                CV_8UC1
            );

            cv::Mat right_bayer(
                height_,
                width_,
                CV_8UC1
            );

            // Read left RAW frame.
            left_raw_file_.read(
                reinterpret_cast<char*>(
                    left_bayer.data),
                static_cast<std::streamsize>(
                    frame_size_)
            );

            if (!left_raw_file_ ||
                left_raw_file_.gcount() !=
                    static_cast<std::streamsize>(frame_size_))
            {
                throw std::runtime_error(
                    "Failed reading left RAW frame " +
                    std::to_string(index));
            }

            // Read right RAW frame.
            right_raw_file_.read(
                reinterpret_cast<char*>(
                    right_bayer.data),
                static_cast<std::streamsize>(
                    frame_size_)
            );

            if (!right_raw_file_ ||
                right_raw_file_.gcount() !=
                    static_cast<std::streamsize>(frame_size_))
            {
                throw std::runtime_error(
                    "Failed reading right RAW frame " +
                    std::to_string(index));
            }

            // Convert BayerRG8 to BGR8.
            cv::Mat left_bgr;
            cv::Mat right_bgr;

            cv::cvtColor(
                left_bayer,
                left_bgr,
                cv::COLOR_BayerRG2BGR
            );

            cv::cvtColor(
                right_bayer,
                right_bgr,
                cv::COLOR_BayerRG2BGR
            );

            // Resize both images.
            cv::resize(
                left_bgr,
                pair.left,
                cv::Size(1280, 800),
                0,
                0,
                cv::INTER_AREA
            );

            cv::resize(
                right_bgr,
                pair.right,
                cv::Size(1280, 800),
                0,
                0,
                cv::INTER_AREA
            );

            const double decode_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() -
                    decode_start
                ).count();

            total_decode_ms += decode_ms;

            max_decode_ms = std::max(
                max_decode_ms,
                decode_ms
            );

            // Wait for available buffer capacity.
            {
                std::unique_lock<std::mutex> lock(
                    buffer_mutex_);

                buffer_condition_.wait(
                    lock,
                    [this]()
                    {
                        return stop_requested_.load() ||
                            frame_buffer_.size() <
                            static_cast<size_t>(buffer_size_);
                    });

                if (stop_requested_.load())
                {
                    break;
                }

                frame_buffer_.push_back(
                    std::move(pair)
                );
            }

            ++decoded;

            buffer_condition_.notify_all();

            if (decoded % 30 == 0)
            {
                RCLCPP_DEBUG(
                    get_logger(),
                    "Decoded %lu stereo pairs. "
                    "Average decode: %.2f ms.",
                    static_cast<unsigned long>(decoded),
                    total_decode_ms / decoded
                );
            }
        }
    }
    catch (const std::exception & error)
    {
        RCLCPP_ERROR(
            get_logger(),
            "RAW decoder error: %s",
            error.what()
        );
    }

    // Record decoder statistics.
    stats_.decoder_end =
        std::chrono::steady_clock::now();

    stats_.decoded_pairs = decoded;

    stats_.total_decode_ms = total_decode_ms;

    stats_.max_decode_ms = max_decode_ms;

    {
        std::lock_guard<std::mutex> lock(
            buffer_mutex_);

        decoder_finished_ = true;
    }

    buffer_condition_.notify_all();

    const double elapsed =
        std::chrono::duration<double>(
            stats_.decoder_end -
            stats_.decoder_start
        ).count();

    RCLCPP_INFO(
        get_logger(),
        "RAW decoder finished: %lu stereo pairs, "
        "%.2f pairs/s, average %.2f ms, max %.2f ms.",
        static_cast<unsigned long>(decoded),
        elapsed > 0.0 ? decoded / elapsed : 0.0,
        decoded > 0 ? total_decode_ms / decoded : 0.0,
        max_decode_ms
    );
}



// ======================================================
// Create ROS image message
// ======================================================

std::unique_ptr<sensor_msgs::msg::Image>
ReplayVideoFileInputNode::create_image_message(
    const cv::Mat & image,
    const FrameTimestamp & timestamp,
    const std::string & frame_id)
{
    if (image.empty()) {
        throw std::runtime_error(
            "Cannot publish empty image.");
    }

    if (timestamp.unix_ns >
        static_cast<uint64_t>(
            std::numeric_limits<int64_t>::max()))
    {
        throw std::overflow_error(
            "Unix timestamp exceeds ROS time range.");
    }

    // Preserve original Unix timestamp.
    const int64_t unix_ns =
        static_cast<int64_t>(timestamp.unix_ns);

    const int64_t seconds =
        unix_ns / 1000000000LL;

    const int64_t nanoseconds =
        unix_ns % 1000000000LL;

    auto message =
        std::make_unique<sensor_msgs::msg::Image>();

    message->header.stamp.sec =
        static_cast<int32_t>(seconds);

    message->header.stamp.nanosec =
        static_cast<uint32_t>(nanoseconds);

    message->header.frame_id = frame_id;

    message->height =
        static_cast<uint32_t>(image.rows);

    message->width =
        static_cast<uint32_t>(image.cols);

    message->encoding = "bgr8";

    message->is_bigendian = false;

    message->step =
        static_cast<uint32_t>(
            image.cols * image.elemSize());

    const size_t data_size =
        image.total() * image.elemSize();

    message->data.resize(data_size);

    if (image.isContinuous()) {

        std::memcpy(
            message->data.data(),
            image.data,
            data_size);
    }
    else {

        const size_t row_size =
            static_cast<size_t>(image.cols) *
            image.elemSize();

        for (int row = 0; row < image.rows; ++row) {

            std::memcpy(
                message->data.data() +
                    static_cast<size_t>(row) * row_size,
                image.ptr(row),
                row_size);
        }
    }

    return message;
}


// ======================================================
// Fixed-FPS publishing
// ======================================================

void ReplayVideoFileInputNode::publish_next_pair()
{
    const auto callback_start =
        std::chrono::steady_clock::now();

    StereoFramePair pair;

    // --------------------------------------------------
    // Retrieve next stereo pair
    // --------------------------------------------------

    {
        std::lock_guard<std::mutex> lock(
            buffer_mutex_);

        if (frame_buffer_.empty()) {

            ++stats_.buffer_empty_events;

            if (decoder_finished_ &&
                !summary_printed_)
            {
                summary_printed_ = true;

                playback_timer_->cancel();

                print_performance_summary();

                RCLCPP_INFO(
                    get_logger(),
                    "End of stereo replay reached.");
            }

            return;
        }

        pair = std::move(frame_buffer_.front());

        frame_buffer_.pop_front();
    }

    buffer_condition_.notify_one();

    // --------------------------------------------------
    // Convert images into ROS messages
    // --------------------------------------------------

    const auto conversion_start =
        std::chrono::steady_clock::now();

    auto left_message =
        create_image_message(
            pair.left,
            pair.left_timestamp,
            "left_camera");

    auto right_message =
        create_image_message(
            pair.right,
            pair.right_timestamp,
            "right_camera");

    const double conversion_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() -
            conversion_start).count();

    // --------------------------------------------------
    // Publish both images
    // --------------------------------------------------

    const auto publish_start =
        std::chrono::steady_clock::now();

    left_publisher_->publish(
        std::move(left_message));

    right_publisher_->publish(
        std::move(right_message));

    const auto publish_end =
        std::chrono::steady_clock::now();

    const double publish_ms =
        std::chrono::duration<double, std::milli>(
            publish_end - publish_start).count();

    const double callback_ms =
        std::chrono::duration<double, std::milli>(
            publish_end - callback_start).count();

    // --------------------------------------------------
    // Update performance statistics
    // --------------------------------------------------

    stats_.total_conversion_ms += conversion_ms;

    stats_.total_publish_ms += publish_ms;

    stats_.total_callback_ms += callback_ms;

    stats_.max_conversion_ms =
        std::max(
            stats_.max_conversion_ms,
            conversion_ms);

    stats_.max_publish_ms =
        std::max(
            stats_.max_publish_ms,
            publish_ms);

    stats_.max_callback_ms =
        std::max(
            stats_.max_callback_ms,
            callback_ms);

    if (!stats_.has_published) {

        stats_.first_publish = publish_end;

        stats_.previous_publish = publish_end;

        stats_.has_published = true;
    }
    else {

        const double interval_ms =
            std::chrono::duration<double, std::milli>(
                publish_end -
                stats_.previous_publish).count();

        stats_.total_interval_ms += interval_ms;

        stats_.max_interval_ms =
            std::max(
                stats_.max_interval_ms,
                interval_ms);

        stats_.previous_publish = publish_end;
    }

    stats_.last_publish = publish_end;

    ++stats_.published_pairs;

    // --------------------------------------------------
    // Periodic diagnostics
    // --------------------------------------------------

    if (stats_.published_pairs % 30 == 0) {

        size_t current_buffer_size;

        {
            std::lock_guard<std::mutex> lock(
                buffer_mutex_);

            current_buffer_size =
                frame_buffer_.size();
        }

        RCLCPP_INFO(
            get_logger(),
            "Published: %lu | Frame: %lu | "
            "Conversion: %.2f ms | Publish: %.2f ms | "
            "Callback: %.2f ms | Buffer: %zu/%d",
            static_cast<unsigned long>(
                stats_.published_pairs),
            static_cast<unsigned long>(
                pair.frame_index),
            conversion_ms,
            publish_ms,
            callback_ms,
            current_buffer_size,
            buffer_size_);
    }
}


// ======================================================
// Performance summary
// ======================================================

void ReplayVideoFileInputNode::print_performance_summary()
{
    const double count =
        static_cast<double>(stats_.published_pairs);

    const double elapsed =
        count > 1.0
        ? std::chrono::duration<double>(
            stats_.last_publish -
            stats_.first_publish).count()
        : 0.0;

    const double actual_fps =
        elapsed > 0.0
        ? (count - 1.0) / elapsed
        : 0.0;

    const double decoder_elapsed =
        std::chrono::duration<double>(
            stats_.decoder_end -
            stats_.decoder_start).count();

    RCLCPP_INFO(
        get_logger(),

        "\n"
        "========== REPLAY PERFORMANCE ==========\n"
        "Decoded pairs:       %lu\n"
        "Published pairs:     %lu\n"
        "Expected pairs:      %lu\n"
        "Actual FPS:          %.2f\n"
        "Target FPS:          %.2f\n"
        "Playback duration:   %.3f s\n"
        "Decoder duration:    %.3f s\n"
        "Average decode:      %.3f ms\n"
        "Maximum decode:      %.3f ms\n"
        "Average conversion:  %.3f ms\n"
        "Maximum conversion:  %.3f ms\n"
        "Average publish:     %.3f ms\n"
        "Maximum publish:     %.3f ms\n"
        "Average callback:    %.3f ms\n"
        "Maximum callback:    %.3f ms\n"
        "Average interval:    %.3f ms\n"
        "Maximum interval:    %.3f ms\n"
        "Empty buffer events: %lu\n"
        "========================================",

        static_cast<unsigned long>(
            stats_.decoded_pairs),

        static_cast<unsigned long>(
            stats_.published_pairs),

        static_cast<unsigned long>(
            frame_count_ - start_frame_),

        actual_fps,

        playback_fps_,

        elapsed,

        decoder_elapsed,

        stats_.decoded_pairs > 0
            ? stats_.total_decode_ms /
                stats_.decoded_pairs
            : 0.0,

        stats_.max_decode_ms,

        count > 0.0
            ? stats_.total_conversion_ms / count
            : 0.0,

        stats_.max_conversion_ms,

        count > 0.0
            ? stats_.total_publish_ms / count
            : 0.0,

        stats_.max_publish_ms,

        count > 0.0
            ? stats_.total_callback_ms / count
            : 0.0,

        stats_.max_callback_ms,

        count > 1.0
            ? stats_.total_interval_ms / (count - 1.0)
            : 0.0,

        stats_.max_interval_ms,

        static_cast<unsigned long>(
            stats_.buffer_empty_events));
}


// ======================================================
// Main
// ======================================================

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    try {

        auto node =
            std::make_shared<ReplayVideoFileInputNode>();

        rclcpp::spin(node);
    }
    catch (const std::exception & error) {

        RCLCPP_ERROR(
            rclcpp::get_logger(
                "replay_video_file_input_node"),
            "Node failed: %s",
            error.what());
    }

    rclcpp::shutdown();

    return 0;
}