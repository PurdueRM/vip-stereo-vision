#include <memory>

#include "rclcpp/rclcpp.hpp"

class ReplayVideoFileInputNode : public rclcpp::Node
{
public:
    ReplayVideoFileInputNode()
        : Node("replay_video_file_input_node")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "'Replay Video File Input Node' initialized."
        );
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<ReplayVideoFileInputNode>()
    );

    rclcpp::shutdown();

    return 0;
}
