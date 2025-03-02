#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace pid
{
  class pid : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    explicit pid(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~pid() override;

  private:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

    void update();
    void getParameters();
    void configPubSub();
    void configTimers();

    rclcpp_lifecycle::LifecyclePublisher<xbox_msgs::msg::Controller>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    float _rate_;
    float _kp_;
    float _kd_;
    float _ki_;


  };

} // namespace xbox_controller_node

