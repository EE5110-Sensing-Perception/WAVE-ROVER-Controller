#include <RobotController.hpp>

#include <UARTSerialPort.hpp>
#include <ROS2Subscriber.hpp>
#include <json.hpp>
#include <RoverCommands.hpp>

#include <algorithm>
#include <iostream>

RobotController::RobotController()
{
    RCLCPP_INFO(rclcpp::get_logger("wave_rover_controller"), "v2.0.0 (cmd_vel driver)");

    _pROS2Subscriber = std::make_shared<ROS2Subscriber>();

    _executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    _execThread = std::make_unique<std::thread>(&RobotController::RunRos2Exectutor, this);

    std::string uart_address = _pROS2Subscriber->get_parameter("UART_address").as_string();

    _wheel_separation = static_cast<float>(_pROS2Subscriber->get_parameter("wheel_separation").as_double());
    _spin_boost = static_cast<float>(_pROS2Subscriber->get_parameter("spin_boost").as_double());
    _motor_speed_max = static_cast<float>(_pROS2Subscriber->get_parameter("motor_speed_max").as_double());
    _linear_clamp_max = static_cast<float>(_pROS2Subscriber->get_parameter("linear_clamp_max").as_double());
    _angular_clamp_max = static_cast<float>(_pROS2Subscriber->get_parameter("angular_clamp_max").as_double());
    _cmd_timeout_ms = _pROS2Subscriber->get_parameter("cmd_timeout_ms").as_int();
    _send_min_interval_ms = _pROS2Subscriber->get_parameter("send_min_interval_ms").as_int();

    if (uart_address.empty()) {
        uart_address = "/dev/ttyUSB0";
    }

    const float speed_scale = static_cast<float>(_pROS2Subscriber->get_parameter("speed_scale").as_double());
    RCLCPP_INFO(
        _pROS2Subscriber->get_logger(),
        "Config: UART=%s speed_scale=%.2f wheel_separation=%.3f "
        "spin_boost=%.2f motor_max=%.2f linear_clamp=%.2f angular_clamp=%.2f",
        uart_address.c_str(), speed_scale, _wheel_separation, _spin_boost,
        _motor_speed_max, _linear_clamp_max, _angular_clamp_max);

    _pROS2Subscriber->SubscribeToTopic("/cmd_vel", [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        SendCmdVel(msg);
    });

    _pUARTSerialPort = std::make_shared<UARTSerialPort>(uart_address, 115200);

    _last_current_debug_row = -1;

    const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    _lastCmdTimeMs.store(now_ms);
    _lastSendMs.store(0);
    _robotMoving.store(false);
    _running.store(true);

    _watchdogThread = std::thread([this]() {
        while (_running.load()) {
            const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();

            const long long last_ms = _lastCmdTimeMs.load();
            const long long elapsed = now_ms - last_ms;

            if (elapsed > _cmd_timeout_ms) {
                auto stop_msg = std::make_shared<geometry_msgs::msg::Twist>();
                stop_msg->linear.x = 0.0;
                stop_msg->angular.z = 0.0;
                SendCmdVel(stop_msg, false);
                _robotMoving.store(false);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
    });

    RCLCPP_INFO(_pROS2Subscriber->get_logger(), "Subscribed to /cmd_vel — ready for joystick, keyboard, or autonomy.");
}

RobotController::~RobotController()
{
    _running = false;
    if (_watchdogThread.joinable()) {
        _watchdogThread.join();
    }

    SendEmergencyStop();

    if (_executor) {
        _executor->cancel();
    }
    if (_execThread && _execThread->joinable()) {
        _execThread->join();
    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    RCLCPP_INFO(rclcpp::get_logger("wave_rover_controller"), "ROS2 node shut down.");
}

void RobotController::RunRos2Exectutor()
{
    std::cout << "STARTING EXECUTOR" << std::endl;
    _executor->add_node(_pROS2Subscriber);
    _executor->spin();
    _executor->remove_node(_pROS2Subscriber);
}

bool RobotController::DisplayMessage(
    int seconds, const std::string & line_1, const std::string & line_2,
    const std::string & line_3, const std::string & line_4)
{
    ResetOled();
    SetOled(0, line_1);
    SetOled(1, line_2);
    SetOled(2, line_3);
    SetOled(3, line_4);

    std::this_thread::sleep_for(std::chrono::seconds(seconds));
    return ResetOled();
}

bool RobotController::DisplayRollingMessage(const std::string & line)
{
    ResetOled();
    _last_current_debug_row++;
    if (_last_current_debug_row > 3) {
        _last_current_debug_row = 0;
    }
    return SetOled(_last_current_debug_row, line);
}

bool RobotController::SendCmdVel(geometry_msgs::msg::Twist::SharedPtr msg, bool update_timestamp)
{
    if (update_timestamp) {
        const long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        _lastCmdTimeMs.store(now_ms);
    }

    float X = static_cast<float>(msg->linear.x);
    float Z = static_cast<float>(msg->angular.z);

    const float speed_scale = static_cast<float>(_pROS2Subscriber->get_parameter("speed_scale").as_double());
    X = std::clamp(X, -_linear_clamp_max, _linear_clamp_max) * speed_scale;
    Z = std::clamp(Z, -1.0f, _angular_clamp_max) * speed_scale;

    _robotMoving.store((std::abs(X) > 1e-3f) || (std::abs(Z) > 1e-3f));

    auto executed_msg = std::make_shared<geometry_msgs::msg::Twist>();
    executed_msg->linear.x = X;
    executed_msg->angular.z = Z;
    _pROS2Subscriber->PublishCmdVel(executed_msg);

    const long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const long long last_send = _lastSendMs.load();
    if (now_ms - last_send < _send_min_interval_ms) {
        return true;
    }
    _lastSendMs.store(now_ms);

    nlohmann::json message_json;

    message_json["T"] = WAVE_ROVER_COMMAND_TYPE::SPEED_INPUT;
    float left_speed = X - Z * _wheel_separation / 2;
    float right_speed = X + Z * _wheel_separation / 2;
    left_speed = std::clamp(left_speed, -_motor_speed_max, _motor_speed_max);
    right_speed = std::clamp(right_speed, -_motor_speed_max, _motor_speed_max);

    if (std::abs(X) < 1e-3f && std::abs(Z) > 1e-3f) {
        left_speed = (left_speed < 0) ? left_speed - _spin_boost : left_speed + _spin_boost;
        right_speed = (right_speed < 0) ? right_speed - _spin_boost : right_speed + _spin_boost;
    }

    message_json["L"] = left_speed;
    message_json["R"] = right_speed;

    const std::string command = message_json.dump() + "\n";
    _pUARTSerialPort->sendRequestSync(command);

    return true;
}

bool RobotController::SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command, std::string & response)
{
    nlohmann::json message_json = {};
    message_json["T"] = command;
    const std::string cmd = message_json.dump() + "\n";
    return _pUARTSerialPort->getResponseSync(cmd, response);
}

bool RobotController::SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command)
{
    nlohmann::json message_json = {};
    message_json["T"] = command;
    const std::string cmd = message_json.dump() + "\n";
    _pUARTSerialPort->sendRequestSync(cmd);
    return true;
}

bool RobotController::EnableWifiHotspot()
{
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::WIFI_AP_DEFAULT);
}

bool RobotController::ScanWifi()
{
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::WIFI_SCAN);
}

bool RobotController::DisableWifi()
{
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::WIFI_OFF);
}

bool RobotController::SendEmergencyStop()
{
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::EMERGENCY_STOP);
}

bool RobotController::EmergencyStop()
{
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::EMERGENCY_STOP);
}

bool RobotController::SetOled(int row, const std::string & content)
{
    if (row > 3) {
        RCLCPP_WARN(
            _pROS2Subscriber->get_logger(), "Cannot print anything on OLED row %d", row);
        return false;
    }

    nlohmann::json message_json = {};
    message_json["T"] = WAVE_ROVER_COMMAND_TYPE::OLED_SET;
    message_json["lineNum"] = row;
    message_json["Text"] = content;
    const std::string cmd = message_json.dump() + "\n";
    _pUARTSerialPort->sendRequestSync(cmd);
    return true;
}

bool RobotController::ResetOled()
{
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::OLED_DEFAULT);
}

bool RobotController::GetInformation(INFO_TYPE info_type, std::string & response)
{
    WAVE_ROVER_COMMAND_TYPE target_type;
    switch (info_type) {
        case INFO_TYPE::IMU: target_type = WAVE_ROVER_COMMAND_TYPE::IMU_INFO; break;
        case INFO_TYPE::DEVICE: target_type = WAVE_ROVER_COMMAND_TYPE::DEVICE_INFO; break;
        case INFO_TYPE::INA219: target_type = WAVE_ROVER_COMMAND_TYPE::INA219_INFO; break;
        case INFO_TYPE::WIFI: target_type = WAVE_ROVER_COMMAND_TYPE::WIFI_INFO; break;
    }
    return SendGenericCmd(target_type, response);
}
