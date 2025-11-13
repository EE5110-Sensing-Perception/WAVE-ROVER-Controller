#pragma once
#include <QString>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <std_msgs/msg/string.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <QDebug>

class ROS2Subscriber : public rclcpp::Node
{
public:
    ROS2Subscriber();
    ~ROS2Subscriber();



    bool SubscribeToTopic(const QString& topic, std::function<void(const geometry_msgs::msg::Twist::SharedPtr)> callback);

    // Publish velocity commands for other ROS2 nodes to use
    void PublishCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);

private:

      void Start();

      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _twist_subscriptions;

      // Publisher for executed velocity commands
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_publisher;

      // liveness probe
      void LivenessCallback();
      rclcpp::TimerBase::SharedPtr _liveness_timer;
      rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr _liveness_publisher;
      size_t count_;
};
