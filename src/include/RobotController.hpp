#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <RoverCommands.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class UARTSerialPort;
class ROS2Subscriber;

class RobotController {
public:
    RobotController();
    ~RobotController();

    bool EnableWifiHotspot();
    bool DisableWifi();
    bool ScanWifi();
    bool EmergencyStop();
    bool SendEmergencyStop();
    bool SetOled(int row, const std::string & content);
    bool ResetOled();
    bool GetInformation(INFO_TYPE info_type, std::string & response);
    bool SendCmdVel(geometry_msgs::msg::Twist::SharedPtr cmd_vel, bool update_timestamp = true);

private:
    std::shared_ptr<UARTSerialPort> _pUARTSerialPort;
    std::shared_ptr<ROS2Subscriber> _pROS2Subscriber;
    std::unique_ptr<std::thread> _execThread;
    rclcpp::Executor::SharedPtr _executor;

    void RunRos2Exectutor();
    bool SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command, std::string & result);
    bool SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command);
    bool DisplayMessage(int seconds, const std::string & line_1, const std::string & line_2,
        const std::string & line_3, const std::string & line_4);
    bool DisplayRollingMessage(const std::string & line);

    int _last_current_debug_row;

    std::atomic<long long> _lastCmdTimeMs{0};
    std::atomic<long long> _lastSendMs{0};

    std::atomic<bool> _robotMoving{false};
    std::thread _watchdogThread;
    std::atomic<bool> _running{false};

    float _wheel_separation = 2.0f;
    float _spin_boost = 0.25f;
    float _motor_deadband = 0.0f;
    float _motor_speed_max = 0.5f;
    float _linear_clamp_max = 1.0f;
    float _angular_clamp_max = 2.0f;
    int _cmd_timeout_ms = 500;
    int _send_min_interval_ms = 50;
};
