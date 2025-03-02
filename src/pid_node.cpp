#include "PIDviewer/pid_node.hpp"

namespace pid
{
    pid::pid(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("pid_node", options)
    {
        declare_parameter("rate", rclcpp::ParameterValue(10.0));
        declare_parameter("KP", rclcpp::ParameterValue(0.0));
        declare_parameter("KD", rclcpp::ParameterValue(0.0));
        declare_parameter("KI", rclcpp::ParameterValue(0.0));
    }

    CallbackReturn xbox_controller_node::on_configure(const rclcpp_lifecycle::State &)
    {
        getParameters();
        configPubSub();
        configTimers();

        return CallbackReturn::SUCCESS;
    }
    CallbackReturn xbox_controller_node::on_activate(const rclcpp_lifecycle::State &)
    {
        publisher_->on_activate();


        return CallbackReturn::SUCCESS;
    }
    CallbackReturn xbox_controller_node::on_deactivate(const rclcpp_lifecycle::State &)
    {
        publisher_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn xbox_controller_node::on_cleanup(const rclcpp_lifecycle::State &)
    {
        publisher_.reset();
        timer_.reset();
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn xbox_controller_node::on_shutdown(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }
    void xbox_controller_node::update()
    {
        publisher_->publish(data);
    }
    void xbox_controller_node::configPubSub()
    {
        publisher_ = this->create_publisher<xbox_msgs::msg::Controller>("/xbox_controller", 10);
    }
    void xbox_controller_node::configTimers()
    {
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_), std::bind(&xbox_controller_node::update, this));
    }
    void xbox_controller_node::getParameters()
    {
        get_parameter("rate", _rate_);
        get_parameter("KP", _kp_);
        get_parameter("KD", _kd_);
        get_parameter("KI", _ki_);
    }
} // namespace xbox_controller_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pid::pix)
