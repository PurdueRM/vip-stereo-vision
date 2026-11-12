#include <memory>

#include "rclcpp/rclcpp.hpp"

class ArmorIsolationNode : public rclcpp::Node
{
public:
    ArmorIsolationNode()
        : Node("armor_isolation_node")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "'Armor Isolation Node' initialized."
        );
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<ArmorIsolationNode>()
    );

    rclcpp::shutdown();

    return 0;
}
