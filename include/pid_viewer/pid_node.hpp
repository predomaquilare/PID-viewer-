#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float32.hpp>

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

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time;

    float actual_error;
    float last_error;
    float _rate_;

    float proportional;
    float derivative;
    float integrative;

    float _kp_;
    float _kd_;
    float _ki_;
    float _goal_;
  };

} // namespace pid