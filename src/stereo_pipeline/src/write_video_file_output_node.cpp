#include <memory>

#include "rclcpp/rclcpp.hpp"

class WriteVideoFileOutputNode : public rclcpp::Node
{
public:
    WriteVideoFileOutputNode()
        : Node("write_video_file_output_node")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "'Write Video File Output Node' initialized."
        );
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<WriteVideoFileOutputNode>()
    );

    rclcpp::shutdown();

    return 0;
}
