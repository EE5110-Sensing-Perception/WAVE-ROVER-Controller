#include <RobotController.hpp>
#include <UARTSerialPort.hpp>
#include <ROS2Subscriber.hpp>
#include <json.hpp>
#include <RoverCommands.hpp>
#include <QDebug>
#include <algorithm>
#include <QtConcurrent/QtConcurrent>
#include <JoypadController.hpp>
#include <iostream>

RobotController::RobotController() {
    qDebug() << "v1.0.3";
    _pROS2Subscriber = std::make_shared<ROS2Subscriber>();
    _pROS2Subscriber->declare_parameter("speed_scale", 0.2);  // Default 20% max speed (was 0.05 = 5%)

    _executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    _execThread = std::make_unique<std::thread>(&RobotController::RunRos2Exectutor, this);

    int enable_joypad = _pROS2Subscriber->get_parameter("enable_joypad").as_int();
    std::string UART_address = _pROS2Subscriber->get_parameter("UART_address").as_string();
    
    if(UART_address.empty()) {
        UART_address = "/dev/ttyUSB0";
    }

    qDebug() << "Joypad enabled: " << enable_joypad;
    qDebug() << "UART Address: " << QString::fromStdString(UART_address);

    _pROS2Subscriber->SubscribeToTopic("/cmd_vel", [&](const geometry_msgs::msg::Twist::SharedPtr msg){
        SendCmdVel(msg);
    });

    _pUARTSerialPort = std::make_shared<UARTSerialPort>(QString::fromStdString(UART_address), 115200);
    // Use QueuedConnection to ensure serial port operations happen in main thread
    // QSerialPort uses QSocketNotifier which must be in the same thread as QObject
    // Explicitly cast to QString overload to resolve ambiguity
    QObject::connect(this, &RobotController::SendRequestSync, _pUARTSerialPort.get(), 
                     static_cast<void (UARTSerialPort::*)(QString)>(&UARTSerialPort::sendRequestSync), 
                     Qt::QueuedConnection);

    if(enable_joypad) {
        _pJoypadController = std::make_shared<JoypadController>();
        QObject::connect(_pJoypadController.get(), SIGNAL(JoypadCommandAvailable(TimestampedDouble, TimestampedDouble)),
                         this, SLOT(JoypadCommandReceived(TimestampedDouble, TimestampedDouble)));
        _pJoypadController->start();
    }

    _last_current_debug_row = -1;

    // Store now in ms
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now().time_since_epoch()).count();
    _lastCmdTimeMs.store(now_ms);
    _lastSendMs.store(0);
    _robotMoving.store(false);
    _running.store(true);

    // Watchdog thread - applies smooth decay when no commands received
    _watchdogThread = std::thread([this]() {
        while (_running.load()) {
            auto now = std::chrono::steady_clock::now();
            long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()).count();

            long long last_ms = _lastCmdTimeMs.load();
            long long elapsed = now_ms - last_ms;

            // If no commands received for a while, smoothly decay towards zero
            if (elapsed > CMD_TIMEOUT_MS) {
                // Send zero target and let smoothing naturally bring values down
                // This creates a smooth ramp-down instead of abrupt stop
                auto decay_msg = std::make_shared<geometry_msgs::msg::Twist>();
                decay_msg->linear.x = 0.0;
                decay_msg->angular.z = 0.0;
                SendCmdVel(decay_msg, /*update_timestamp=*/false);
                
                // Check if we're effectively stopped
                bool moving = (std::abs(_lastLinearX) > 1e-3f) || (std::abs(_lastAngularZ) > 1e-3f);
                _robotMoving.store(moving);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
    });


    qDebug() << "Initialization sequence...";
}

RobotController::~RobotController() {
    _running = false;
    if (_watchdogThread.joinable()) _watchdogThread.join();

    SendEmergencyStop();
    rclcpp::shutdown();
    _executor->cancel();
    if (_execThread->joinable()) {
        _execThread->join();
    }
    qDebug() << "ROS2 Node shut down.";
}

void RobotController::RunRos2Exectutor() {
    std::cout << "STARTING EXECUTOR" << std::endl;
    _executor->add_node(_pROS2Subscriber);
    _executor->spin();
    _executor->remove_node(_pROS2Subscriber);
}

bool RobotController::DisplayMessage(int seconds, QString line_1, QString line_2, QString line_3, QString line_4){
    ResetOled();
    SetOled(0, line_1);
    SetOled(1, line_2);
    SetOled(2, line_3);
    SetOled(3, line_4);

    std::this_thread::sleep_for(std::chrono::seconds(seconds));
    return ResetOled();
}

bool RobotController::DisplayRollingMessage(QString line){
    ResetOled();
    _last_current_debug_row++;
    if(_last_current_debug_row > 3) _last_current_debug_row = 0;
    return SetOled(_last_current_debug_row, line);
}

bool RobotController::SendCmdVel(geometry_msgs::msg::Twist::SharedPtr msg, bool update_timestamp) {
    // Only update last-cmd time when this is a "real" incoming command (not watchdog stop)
    if (update_timestamp) {
        long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        _lastCmdTimeMs.store(now_ms);
    }

    // Exponential smoothing of target commands with improved responsiveness
    float targetX = msg->linear.x;
    float targetZ = msg->angular.z;

    _lastLinearX = _lastLinearX + SMOOTHING_ALPHA * (targetX - _lastLinearX);
    _lastAngularZ = _lastAngularZ + SMOOTHING_ALPHA * (targetZ - _lastAngularZ);

    bool moving = (std::abs(_lastLinearX) > 1e-3f) || (std::abs(_lastAngularZ) > 1e-3f);
    _robotMoving.store(moving);

    // Use teleop scaling (speed_scale remains a global cap)
    float speed_scale = _pROS2Subscriber->get_parameter("speed_scale").as_double();

    // Clamp input values for safety
    float x = std::clamp(_lastLinearX, -1.0f, 1.0f);
    float z = std::clamp(_lastAngularZ, -1.0f, 2.0f);

    // Differential drive mapping: converts linear (x) and angular (z) velocities to wheel speeds
    // Standard kinematic formula: left = x - z, right = x + z (for positive z = CCW rotation)
    // Current implementation uses: left = x + z, right = x - z
    // NOTE: If turn direction is wrong, swap the signs here or swap L/R assignment below
    // Alternative formula (from provided code): left = x - z, right = x + z (then conditionally swapped)
    // Motor controller expects values in range -1.0 to 1.0 (NOT -255 to 255)
    float left = speed_scale * (x - z);
    float right = speed_scale * (x + z);

    // Clamp final motor values to valid range (-1.0 to 1.0)
    left = std::clamp(left, -1.0f, 1.0f);
    right = std::clamp(right, -1.0f, 1.0f);
    
    // Apply minimum threshold to avoid motor dead zone (motors may ignore very small values)
    // If absolute value is below threshold, set to zero instead
    const float MIN_MOTOR_THRESHOLD = 0.02f;  // Minimum value motor controller will respond to (2% of range)
    if (std::abs(left) < MIN_MOTOR_THRESHOLD) left = 0.0f;
    if (std::abs(right) < MIN_MOTOR_THRESHOLD) right = 0.0f;

    // Rate-limit UART writes to avoid flooding the serial link
    long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    long long last_send = _lastSendMs.load();
    if (now_ms - last_send < SEND_MIN_INTERVAL_MS) {
        // too soon: skip sending but internal state updated
        return true;
    }
    _lastSendMs.store(now_ms);

    nlohmann::json message_json;
    message_json["T"] = WAVE_ROVER_COMMAND_TYPE::SPEED_INPUT;
    // Motor controller expects float values in range -1.0 to 1.0
    message_json["L"] = left;
    message_json["R"] = right;

    // Debug output to verify speed values (can be removed in production)
    qDebug() << "Speed cmd - L:" << left << "R:" << right 
             << "(scale:" << speed_scale << "x:" << x << "z:" << z << ")";

    QString command = QString::fromStdString(message_json.dump()) + "\n";
    emit SendRequestSync(command);
    return true;
}



void RobotController::JoypadCommandReceived(TimestampedDouble t1, TimestampedDouble t2) {
    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = t1.value;
    msg->angular.z = t2.value;
    SendCmdVel(msg);
}

bool RobotController::SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command, QString& response){
    nlohmann::json message_json = {};
    message_json["T"] = command;
    // FIX: Add newline to generic commands too
    QString cmd = QString::fromStdString(message_json.dump()) + "\n";
    return _pUARTSerialPort->getResponseSync(cmd, response);
}

bool RobotController::SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command){
    nlohmann::json message_json = {};
    message_json["T"] = command;
    // FIX: Add newline to generic commands too
    QString cmd = QString::fromStdString(message_json.dump()) + "\n";
    _pUARTSerialPort->sendRequestSync(cmd);
    return true;
}

bool RobotController::EnableWifiHotspot(){
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::WIFI_AP_DEFAULT);
}

bool RobotController::ScanWifi(){
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::WIFI_SCAN);
}

bool RobotController::DisableWifi(){
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::WIFI_OFF);
}

bool RobotController::SendEmergencyStop() {
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::EMERGENCY_STOP);
}

bool RobotController::EmergencyStop(){
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::EMERGENCY_STOP);
}

bool RobotController::SetOled(int row, QString content){
    if(row > 3)
    {
        qDebug() << "Cannot print anything on OLED row " << row;
        return false;
    }

    nlohmann::json message_json = {};
    message_json["T"] = WAVE_ROVER_COMMAND_TYPE::OLED_SET;
    message_json["lineNum"] = row;
    message_json["Text"] = content.toStdString();
    // FIX: Add newline to OLED commands too
    QString cmd = QString::fromStdString(message_json.dump()) + "\n";
    _pUARTSerialPort->sendRequestSync(cmd);
    return true;
}

bool RobotController::ResetOled() {
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::OLED_DEFAULT);
}

bool RobotController::GetInformation(INFO_TYPE info_type, QString& response){
    WAVE_ROVER_COMMAND_TYPE target_type;
    switch (info_type)
    {
    case INFO_TYPE::IMU:
        target_type = WAVE_ROVER_COMMAND_TYPE::IMU_INFO;
        break;

    case INFO_TYPE::DEVICE:
        target_type = WAVE_ROVER_COMMAND_TYPE::DEVICE_INFO;
        break;

    case INFO_TYPE::INA219:
        target_type = WAVE_ROVER_COMMAND_TYPE::INA219_INFO;
        break;

    case INFO_TYPE::WIFI:
        target_type = WAVE_ROVER_COMMAND_TYPE::WIFI_INFO;
        break;
    }
    return SendGenericCmd(target_type, response);
}

