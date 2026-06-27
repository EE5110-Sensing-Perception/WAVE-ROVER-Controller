#include <ROS2Subscriber.hpp>

#include <chrono>

using namespace std::chrono_literals;

ROS2Subscriber::ROS2Subscriber()
: Node("wave_rover_controller")
{
    this->declare_parameter("UART_address", rclcpp::ParameterValue(std::string("/dev/ttyUSB0")));
    this->declare_parameter("speed_scale", rclcpp::ParameterValue(0.5));
    this->declare_parameter("wheel_separation", rclcpp::ParameterValue(2.0));
    this->declare_parameter("spin_boost", rclcpp::ParameterValue(0.25));
    this->declare_parameter("motor_deadband", rclcpp::ParameterValue(0.0));
    this->declare_parameter("motor_speed_max", rclcpp::ParameterValue(0.5));
    this->declare_parameter("linear_clamp_max", rclcpp::ParameterValue(1.0));
    this->declare_parameter("angular_clamp_max", rclcpp::ParameterValue(2.0));
    this->declare_parameter("cmd_timeout_ms", rclcpp::ParameterValue(500));
    this->declare_parameter("send_min_interval_ms", rclcpp::ParameterValue(50));

    _liveness_publisher = this->create_publisher<builtin_interfaces::msg::Time>("/liveness", 10);
    _liveness_timer = this->create_wall_timer(
        500ms, std::bind(&ROS2Subscriber::LivenessCallback, this));

    _cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_executed", 10);
    RCLCPP_INFO(this->get_logger(), "ROS2 node initialized");
    RCLCPP_INFO(this->get_logger(), "Velocity command publisher created on topic: /cmd_vel_executed");
}

ROS2Subscriber::~ROS2Subscriber() = default;

bool ROS2Subscriber::SubscribeToTopic(
    const std::string & topic,
    std::function<void(const geometry_msgs::msg::Twist::SharedPtr)> callback)
{
    RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", topic.c_str());
    _twist_subscriptions = this->create_subscription<geometry_msgs::msg::Twist>(
        topic,
        rclcpp::SensorDataQoS().reliable(),
        [callback](const geometry_msgs::msg::Twist::SharedPtr msg) {
            std::cout << " >>> ";
            callback(msg);
        });
    return true;
}

void ROS2Subscriber::LivenessCallback()
{
    builtin_interfaces::msg::Time message;
    const rclcpp::Time time = this->get_clock()->now();
    message.sec = static_cast<int32_t>(time.seconds());
    message.nanosec = static_cast<uint32_t>(time.nanoseconds());
    _liveness_publisher->publish(message);
}

void ROS2Subscriber::PublishCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (_cmd_vel_publisher) {
        _cmd_vel_publisher->publish(*msg);
    }
}
