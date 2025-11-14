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
    qDebug() << "v1.0.6 (XZ control, no smoothing)";
    _pROS2Subscriber = std::make_shared<ROS2Subscriber>();
    _pROS2Subscriber->declare_parameter("speed_scale", 1.0);  // Default 100% max speed

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

    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now().time_since_epoch()).count();
    _lastCmdTimeMs.store(now_ms);
    _lastSendMs.store(0);
    _robotMoving.store(false);
    _running.store(true);

    // Watchdog thread - immediately stops robot if no command received
    _watchdogThread = std::thread([this]() {
        while (_running.load()) {
            auto now = std::chrono::steady_clock::now();
            long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()).count();

            long long last_ms = _lastCmdTimeMs.load();
            long long elapsed = now_ms - last_ms;

            if (elapsed > CMD_TIMEOUT_MS) {
                auto stop_msg = std::make_shared<geometry_msgs::msg::Twist>();
                stop_msg->linear.x = 0.0;
                stop_msg->angular.z = 0.0;
                SendCmdVel(stop_msg, false);

                _robotMoving.store(false);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
    });

    qDebug() << "Initialization sequence complete.";
}

RobotController::~RobotController() {
    _running = false;
    if (_watchdogThread.joinable()) _watchdogThread.join();

    SendEmergencyStop();
    rclcpp::shutdown();
    _executor->cancel();
    if (_execThread->joinable()) _execThread->join();

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

// --- X/Z control without smoothing ---
bool RobotController::SendCmdVel(geometry_msgs::msg::Twist::SharedPtr msg, bool update_timestamp) {
    if (update_timestamp) {
        long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        _lastCmdTimeMs.store(now_ms);
    }

    // Directly use incoming X/Z values (no smoothing)
    float X = msg->linear.x;
    float Z = msg->angular.z;

    // Clamp values
    float speed_scale = _pROS2Subscriber->get_parameter("speed_scale").as_double();
    X = std::clamp(X, -1.0f, 1.0f) * speed_scale;
    Z = std::clamp(Z, -1.0f, 2.0f) * speed_scale;

    _robotMoving.store((std::abs(X) > 1e-3f) || (std::abs(Z) > 1e-3f));

    // Publish for ROS2 visibility
    auto executed_msg = std::make_shared<geometry_msgs::msg::Twist>();
    executed_msg->linear.x = X;
    executed_msg->angular.z = Z;
    _pROS2Subscriber->PublishCmdVel(executed_msg);

    // Rate-limit UART writes
    long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    long long last_send = _lastSendMs.load();
    if (now_ms - last_send < SEND_MIN_INTERVAL_MS) return true;
    _lastSendMs.store(now_ms);

    nlohmann::json message_json;
    message_json["T"] = ROS_CTRL; // ROS2 cmd_vel X/Z
    message_json["X"] = X;
    message_json["Z"] = Z;

    qDebug() << "XZ cmd - X:" << X << " Z:" << Z;

    QString command = QString::fromStdString(message_json.dump()) + "\n";
    emit SendRequestSync(command);

    return true;
}

// --- Joystick ---
void RobotController::JoypadCommandReceived(TimestampedDouble t1, TimestampedDouble t2) {
    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = t1.value;
    msg->angular.z = t2.value;
    SendCmdVel(msg);
}

// --- Generic Commands ---
bool RobotController::SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command, QString& response){
    nlohmann::json message_json = {};
    message_json["T"] = command;
    QString cmd = QString::fromStdString(message_json.dump()) + "\n";
    return _pUARTSerialPort->getResponseSync(cmd, response);
}

bool RobotController::SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command){
    nlohmann::json message_json = {};
    message_json["T"] = command;
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

// --- OLED ---
bool RobotController::SetOled(int row, QString content){
    if(row > 3) {
        qDebug() << "Cannot print anything on OLED row " << row;
        return false;
    }
    nlohmann::json message_json = {};
    message_json["T"] = WAVE_ROVER_COMMAND_TYPE::OLED_SET;
    message_json["lineNum"] = row;
    message_json["Text"] = content.toStdString();
    QString cmd = QString::fromStdString(message_json.dump()) + "\n";
    _pUARTSerialPort->sendRequestSync(cmd);
    return true;
}

bool RobotController::ResetOled() {
    return SendGenericCmd(WAVE_ROVER_COMMAND_TYPE::OLED_DEFAULT);
}

// --- Info queries ---
bool RobotController::GetInformation(INFO_TYPE info_type, QString& response){
    WAVE_ROVER_COMMAND_TYPE target_type;
    switch (info_type) {
        case INFO_TYPE::IMU: target_type = WAVE_ROVER_COMMAND_TYPE::IMU_INFO; break;
        case INFO_TYPE::DEVICE: target_type = WAVE_ROVER_COMMAND_TYPE::DEVICE_INFO; break;
        case INFO_TYPE::INA219: target_type = WAVE_ROVER_COMMAND_TYPE::INA219_INFO; break;
        case INFO_TYPE::WIFI: target_type = WAVE_ROVER_COMMAND_TYPE::WIFI_INFO; break;
    }
    return SendGenericCmd(target_type, response);
}
