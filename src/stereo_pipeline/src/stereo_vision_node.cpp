#include <memory>

#include "rclcpp/rclcpp.hpp"

class StereoVisionNode : public rclcpp::Node
{
public:
    StereoVisionNode()
        : Node("stereo_vision_node")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "'Stereo Vision Node' initialized."
        );
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<StereoVisionNode>()
    );

    rclcpp::shutdown();

    return 0;
}
