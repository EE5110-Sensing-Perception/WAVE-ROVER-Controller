# Wave Rover Controller

ROS 2 driver for the Waveshare Wave Rover.  
Forked from [antlassagne/ros2-wave-rover](https://github.com/antlassagne/ros2-wave-rover), with key updates:

- Updated UART communication
- Added keyboard and joypad control support
- Improved debugging and error handling

This package should also work with other Waveshare robots using JSON/UART communication (Offroad UGV, etc.).

![Wave Rover Robot](images/wave-rover-1.jpg)

---

## Features

- **ROS2 Integration**: Subscribes to `/cmd_vel` and converts `geometry_msgs/Twist` commands to motor control.
- **UART Communication**: JSON-based protocol with the robot’s ESP32.
- **Keyboard & Joypad Support**: Manual control from a remote PC or onboard joystick.
- **OLED Display Control**: Display messages and status on the robot’s OLED.
- **WiFi Management**: Configure the robot’s hotspot and scan networks.
- **Emergency Stop**: Safety mechanism for immediate halt.
- **Multi-threaded Architecture**: Separate threads for ROS2 execution and UART communication.
- **Configurable Parameters**: Adjustable speed scaling and UART device path.

---

## Package Information

- **Package Name**: `wave_rover_controller`
- **Version**: 0.0.0
- **License**: MIT
- **Maintainer**: briandeegan82 (brian.deegan82@gmail.com)

---

## Deployment Setup (Preferred)
Clone this repo onto both remote device and host PC. Run teleop from the host pc, and the controller on the remote device.

### 1. Robot (Driver & UART)

Run the ROS2 driver on the robot to handle UART communication:

```bash
# If using Docker
# build container:
docker build -t wave-rover-controller .

# run controller node
docker run -it --rm \
  --device=/dev/ttyUSB0 \
  --group-add dialout \
  --net=host \
  wave-rover-controller \
  ros2 launch wave_rover_controller wave_rover_launch.py UART_address:="/dev/ttyUSB0"
```

**Note:** `--group-add dialout` ensures serial port access without modifying udev rules.
The robot will subscribe to `/cmd_vel` and execute motor commands.

### 2. Host PC (Keyboard Teleop)

On your host PC, run ROS2 keyboard teleoperation:

```bash
# build and run the same docker container on your host PC
docker build -t wave-rover-host .

# run the container
docker run -it --rm --privileged --net=host --group-add dialout wave-rover-host

# Launch keyboard teleop pointing to the robot's ROS2 network
export ROS_DOMAIN_ID=<match_robot_domain>
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Commands sent from the keyboard will be transmitted to the robot over the ROS2 network.

**note: ensure you have clicked on the teleop terminal, otherwise commands won't be sent**

---

# WARNING!!!!!!
Always turn off the controller before keyboard teleop. The robot will take off if you don't!

## Optional: Joypad Support

Enable joystick control on the robot:

```bash
ros2 launch wave_rover_controller wave_rover_launch.py enable_joypad:=1 UART_address:="/dev/ttyUSB0"
```

---

## ROS2 Topics

### Subscribed Topics

* `/cmd_vel` (`geometry_msgs/msg/Twist`): Velocity commands for robot movement.

### Published Topics

* None (control-only node).

---

## Launch Parameters

### `wave_rover_launch.py`

* `enable_joypad` (default: 0): Enable joystick control.
* `UART_address` (default: `/dev/ttyUSB0`): Serial device path.

### `control_launch.py`

* `require_enable_button` (default: False)
* `axis_linear.x` (default: 4)
* `axis_angular.yaw` (default: 0)
* `scale_linear.x` (default: 1.0)
* `scale_angular.yaw` (default: 1.0)

---

## Hardware Setup

### UART Connection

Connect the ESP32 USB at the bottom of the rover.

![USB Connection](images/usb_serial_connection.jpg)

> Make sure `/dev/ttyUSB0` is accessible (use `--group-add dialout` in Docker).

---

## Testing & Debugging

### Manual Serial Testing

```bash
stty -F /dev/ttyUSB0 115200
cat /dev/ttyUSB0
echo -ne '{"T":-3}\n' > /dev/ttyUSB0       # Reset OLED
echo -ne '{"T":1,"L":120,"R":120}\n' > /dev/ttyUSB0  # Move forward
```

### ROS2 Testing

On host or robot:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x:0.1}, angular: {z:1.0}}"
ros2 topic echo /cmd_vel
```

---

## JSON Protocol Reference

```json
{
  "T": <command_type>,
  "L": <left_motor_speed>,
  "R": <right_motor_speed>
}
```

### Command Types

* `-3`: OLED Reset
* `0`: Emergency Stop
* `1`: Speed Input (`L`/`R`: -255 to 255)
* `3`: OLED Set (`lineNum`, `Text`)

---

## Troubleshooting

* **Serial Permission Denied**: Use `--group-add dialout` or set udev rules.
* **No Response from Robot**: Check `/dev/ttyUSB0` and baud rate (115200).
* **Joypad Not Working**: Verify connection and permissions.
* **Build Errors**: Ensure Qt and ROS2 dependencies are installed.

Debugging tips:

```bash
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG
ls -la /dev/ttyUSB*
journalctl -f
```

---

## Contributing

Based on [antlassagne/ros2-wave-rover](https://github.com/antlassagne/ros2-wave-rover) with:

* Updated UART communication
* Joypad and keyboard support
* Improved logging and debugging
* Enhanced documentation

---

## License

MIT License (see `package.xml` for details)

```
