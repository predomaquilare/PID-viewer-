#include "rclcpp/rclcpp.hpp"
#include "pid_viewer/pid_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pid::pid>(rclcpp::NodeOptions());
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}