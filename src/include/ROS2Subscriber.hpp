#pragma once

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <builtin_interfaces/msg/time.hpp>

class ROS2Subscriber : public rclcpp::Node
{
public:
    ROS2Subscriber();
    ~ROS2Subscriber();

    bool SubscribeToTopic(
        const std::string & topic,
        std::function<void(const geometry_msgs::msg::Twist::SharedPtr)> callback);

    void PublishCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);

private:
    void Start();
    void LivenessCallback();

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _twist_subscriptions;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_publisher;
    rclcpp::TimerBase::SharedPtr _liveness_timer;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr _liveness_publisher;
    size_t count_;
};
