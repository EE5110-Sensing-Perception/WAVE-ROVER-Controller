#include <ROS2Subscriber.hpp>
#include <QDebug>
#include <chrono>

using namespace std::chrono_literals;

ROS2Subscriber::ROS2Subscriber() : rclcpp::Node("WaveRobotController") {
    this->declare_parameter("UART_address", rclcpp::PARAMETER_STRING);
    this->declare_parameter("enable_joypad", rclcpp::PARAMETER_INTEGER);
    _liveness_publisher = this->create_publisher<builtin_interfaces::msg::Time>("/liveness", 10);
    _liveness_timer = this->create_wall_timer(
    500ms, std::bind(&ROS2Subscriber::LivenessCallback, this));
    
    // Create publisher for executed velocity commands
    _cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_executed", 10);
    qDebug() << "ROS2 Node initialized.";
    qDebug() << "Velocity command publisher created on topic: /cmd_vel_executed";
}

ROS2Subscriber::~ROS2Subscriber(){

}

bool ROS2Subscriber::SubscribeToTopic(const QString& topic, std::function<void(const geometry_msgs::msg::Twist::SharedPtr)> callback) {
    qDebug() << "Topic subscription.";
    _twist_subscriptions = this->create_subscription<geometry_msgs::msg::Twist>(
                                                "/cmd_vel", rclcpp::SensorDataQoS().reliable(), [callback](const geometry_msgs::msg::Twist::SharedPtr msg){
        std::cout << " >>> ";
        callback(msg);
    });

    return true;
}

void ROS2Subscriber::LivenessCallback()
{
  builtin_interfaces::msg::Time message;
  rclcpp::Time time = this->get_clock()->now();
  message.sec = time.seconds();
  message.nanosec = time.nanoseconds();
//      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  _liveness_publisher->publish(message);
}

void ROS2Subscriber::PublishCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (_cmd_vel_publisher) {
        _cmd_vel_publisher->publish(*msg);
    }
}

