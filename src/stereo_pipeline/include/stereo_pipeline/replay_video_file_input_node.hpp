
#ifndef STEREO_PIPELINE_REPLAY_VIDEO_FILE_INPUT_NODE_HPP_
#define STEREO_PIPELINE_REPLAY_VIDEO_FILE_INPUT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class ReplayVideoFileInputNode : public rclcpp::Node
{
public:

    ReplayVideoFileInputNode();

    ~ReplayVideoFileInputNode();

private:

    // ==================================================
    // Timestamp structure
    // ==================================================

    struct FrameTimestamp
    {
        uint64_t unix_ns = 0;
        uint64_t monotonic_ns = 0;
    };


    // ==================================================
    // Stereo frame structure
    // ==================================================

    struct StereoFramePair
    {
        uint64_t frame_index = 0;

        FrameTimestamp left_timestamp;
        FrameTimestamp right_timestamp;

        cv::Mat left;
        cv::Mat right;
    };


    // ==================================================
    // Performance statistics
    // ==================================================

    struct PerformanceStats
    {
        uint64_t decoded_pairs = 0;
        uint64_t published_pairs = 0;
        uint64_t buffer_empty_events = 0;

        double total_decode_ms = 0.0;
        double max_decode_ms = 0.0;

        double total_conversion_ms = 0.0;
        double max_conversion_ms = 0.0;

        double total_publish_ms = 0.0;
        double max_publish_ms = 0.0;

        double total_callback_ms = 0.0;
        double max_callback_ms = 0.0;

        double total_interval_ms = 0.0;
        double max_interval_ms = 0.0;

        std::chrono::steady_clock::time_point decoder_start;
        std::chrono::steady_clock::time_point decoder_end;

        std::chrono::steady_clock::time_point first_publish;
        std::chrono::steady_clock::time_point last_publish;
        std::chrono::steady_clock::time_point previous_publish;

        bool has_published = false;
    };


    // ==================================================
    // Parameters
    // ==================================================

    std::string left_video_path_;
    std::string right_video_path_;

    std::string left_timestamps_path_;
    std::string right_timestamps_path_;

    int start_frame_ = 0;
    int buffer_size_ = 8;

    bool reliable_qos_ = true;

    double playback_fps_ = 30.0;

    // ==================================================
    // RAW file handling
    // ==================================================

    std::ifstream left_raw_file_;
    std::ifstream right_raw_file_;

    int width_ = 1920;
    int height_ = 1200;

    uint64_t frame_size_ = 0;
    uint64_t frame_count_ = 0;


    // ==================================================
    // Timestamp storage
    // ==================================================

    std::vector<FrameTimestamp> left_timestamps_;
    std::vector<FrameTimestamp> right_timestamps_;


    // ==================================================
    // ROS publishers
    // ==================================================

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        left_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        right_publisher_;


    // ==================================================
    // Playback timer
    // ==================================================

    rclcpp::TimerBase::SharedPtr playback_timer_;


    // ==================================================
    // Decoder thread
    // ==================================================

    std::thread decoder_thread_;

    std::atomic<bool> stop_requested_{false};

    bool decoder_finished_ = false;


    // ==================================================
    // Frame buffer
    // ==================================================

    std::deque<StereoFramePair> frame_buffer_;

    std::mutex buffer_mutex_;

    std::condition_variable buffer_condition_;


    // ==================================================
    // Performance tracking
    // ==================================================

    PerformanceStats stats_;

    bool summary_printed_ = false;


    // ==================================================
    // Methods
    // ==================================================

    bool initialize_videos();

    bool load_timestamps();

    std::vector<FrameTimestamp> read_timestamp_csv(
        const std::string & path
    );

    void stop_decoder();

    void decoder_worker();

    void publish_next_pair();

    void print_performance_summary();

    std::unique_ptr<sensor_msgs::msg::Image>
    create_image_message(
        const cv::Mat & image,
        const FrameTimestamp & timestamp,
        const std::string & frame_id
    );
};

#endif
