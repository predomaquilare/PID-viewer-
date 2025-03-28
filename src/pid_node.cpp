#include "pid_viewer/pid_node.hpp"
namespace pid
{
    pid::pid(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("pid_node", options)
    {
        declare_parameter("rate", rclcpp::ParameterValue(10.0));
        declare_parameter("kp", rclcpp::ParameterValue(0.0));
        declare_parameter("kd", rclcpp::ParameterValue(0.0));
        declare_parameter("ki", rclcpp::ParameterValue(0.0));
        declare_parameter("goal", rclcpp::ParameterValue(0.0));

        last_time = rclcpp::Time(0);
        last_error = 0;
        proportional = 0;
        derivative = 0;
        integrative = 0;
        actual_error = 0;
    }
    pid::~pid() {}


    CallbackReturn pid::on_configure(const rclcpp_lifecycle::State &)
    {
        getParameters();
        configPubSub();
        configTimers();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn pid::on_activate(const rclcpp_lifecycle::State &)
    {
        publisher_->on_activate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn pid::on_deactivate(const rclcpp_lifecycle::State &)
    {
        publisher_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn pid::on_cleanup(const rclcpp_lifecycle::State &)
    {
        publisher_.reset();
        timer_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn pid::on_shutdown(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

  void pid::update()
  {
      // Obtém o tempo atual a partir do mesmo clock usado anteriormente
       rclcpp::Time now = this->get_clock()->now();
  
    // Garante que 'last_time' tem um valor inicial válido
    if (last_time.nanoseconds() == 0) {
        last_time = now;
    }

    float dt = (now - last_time).seconds();
    float error = _goal_ - actual_error;
    proportional = _kp_ * error;
    derivative = (dt > 0) ? _kd_*((error - last_error) / dt) : 0.0;
    integrative += _ki_ * (error * dt);

    float pid = proportional + derivative + integrative;
    auto msg = std_msgs::msg::Float32();
    msg.data = actual_error;

    actual_error += pid;

    RCLCPP_INFO(get_logger(), "%f, %f, %f" ,error, dt, pid);
    
    
    last_error = error;
    last_time = now; // Atualiza last_time com a mesma fonte de tempo
    publisher_->publish(msg);
}


    void pid::configPubSub()
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/pid_graph", 10);
    }

    void pid::configTimers()
    {
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_), std::bind(&pid::update, this));
    }

    void pid::getParameters()
    {
        get_parameter("rate", _rate_);
        get_parameter("kp", _kp_);
        get_parameter("kd", _kd_);
        get_parameter("ki", _ki_);
        get_parameter("goal", _goal_);
        
        RCLCPP_INFO(get_logger(), "%f" ,_rate_);
        RCLCPP_INFO(get_logger(), "%f" ,_kp_);
        RCLCPP_INFO(get_logger(),"%f" ,_kd_);
        RCLCPP_INFO(get_logger(), "%f" ,_ki_);
        RCLCPP_INFO(get_logger(), "%f" ,_goal_);

    }
} // namespace pid

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pid::pid)
