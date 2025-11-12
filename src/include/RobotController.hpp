#pragma once
#include <QString>
#include <QObject>
#include <memory>
#include <thread>
#include <RoverCommands.hpp>
#include <rclcpp/rclcpp.hpp>
#include <JoypadController.hpp>
#include <atomic>
#include <chrono>

// cmd_vel message type
#include <geometry_msgs/msg/twist.hpp>

class UARTSerialPort;
class ROS2Subscriber;
class JoypadController;

class RobotController: public QObject {
    Q_OBJECT
    public:
        RobotController();
        ~RobotController();

        bool EnableWifiHotspot();
        bool DisableWifi();
        bool ScanWifi();
        bool EmergencyStop();
        bool SendEmergencyStop();
        bool SetOled(int row, QString content);
        bool ResetOled();
        bool GetInformation(INFO_TYPE info_type, QString& response);
        bool SendCmdVel(geometry_msgs::msg::Twist::SharedPtr cmd_vel, bool update_timestamp = true);

public slots:
        void JoypadCommandReceived(TimestampedDouble t1, TimestampedDouble t2);

    signals:
        void SendRequestSync(QString);

    private:
        std::shared_ptr<UARTSerialPort> _pUARTSerialPort;
        std::shared_ptr<JoypadController> _pJoypadController;
        std::shared_ptr<ROS2Subscriber> _pROS2Subscriber;
        std::unique_ptr<std::thread> _execThread;
        rclcpp::Executor::SharedPtr _executor;
        void RunRos2Exectutor();
        bool SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command, QString& result);
        bool SendGenericCmd(WAVE_ROVER_COMMAND_TYPE command);

        bool DisplayMessage(int seconds, QString line_1, QString line_2, QString line_3, QString line_4);
        bool DisplayRollingMessage(QString line);
        int _last_current_debug_row;
        
        // Watchdog timing stored as ms since steady_clock epoch (atomic integral type)
        std::atomic<long long> _lastCmdTimeMs{0};

        // last time we actually wrote to serial (ms)
        std::atomic<long long> _lastSendMs{0};

        // Motion smoothing state
        float _lastLinearX{0.0f};
        float _lastAngularZ{0.0f};

        // Flags and thread
        std::atomic<bool> _robotMoving{false};
        std::thread _watchdogThread;
        std::atomic<bool> _running{false};

        // Tunables
        static constexpr int CMD_TIMEOUT_MS = 500;        // timeout → send stop (increased for smoother operation)
        static constexpr int SEND_MIN_INTERVAL_MS = 50;   // rate limit: ~20 Hz (smoother updates)
        static constexpr float SMOOTHING_ALPHA = 0.15f;   // 0..1 higher = more responsive, still smooth

        // --------------------------------

};


