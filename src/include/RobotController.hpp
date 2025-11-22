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

        // Motion smoothing state (twist-level)
        float _lastLinearX{0.0f};
        float _lastAngularZ{0.0f};

        // Wheel-level last outputs (per-wheel ramping)
        float _lastLeft{0.0f};
        float _lastRight{0.0f};

        // Flags and thread
        std::atomic<bool> _robotMoving{false};
        std::thread _watchdogThread;
        std::atomic<bool> _running{false};

        // Tunables
        static constexpr int CMD_TIMEOUT_MS = 500;        // timeout → send stop (increased for smoother operation)
        static constexpr int SEND_MIN_INTERVAL_MS = 50;   // rate limit: ~20 Hz (smoother updates)
        static constexpr float SMOOTHING_ALPHA = 0.15f;   // 0..1 higher = more responsive, still smooth

        // Wheel-level acceleration limits (units: normalized motor value per second)
        // Tune these: lower = smoother/less aggressive, higher = snappier
        float MAX_WHEEL_DELTA_PER_SEC = 1.0f;            // default general
        float MAX_WHEEL_DELTA_TURNING_PER_SEC = 1.8f;    // allowed when in-place turning

        // In-place turning scale (reduce overall speed when linear.x ~= 0)
        float TURNING_SPEED_SCALE = 0.8f;
        float INPLACE_LINEAR_EPS = 1e-3f; // threshold to treat as in-place turn

        // Motor deadzone threshold
        static constexpr float MIN_MOTOR_THRESHOLD = 0.02f;  // 2% of range

        // Debug / config could be promoted to ros params if desired
        
        // --------------------------------

};