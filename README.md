# Wave Rover Controller

ROS 2 driver for the Waveshare Wave Rover.  
Forked from [antlassagne/ros2-wave-rover](https://github.com/antlassagne/ros2-wave-rover).

The driver subscribes to **`/cmd_vel`** and converts `geometry_msgs/Twist` to motor commands over UART.  
Joystick, keyboard, and autonomous stacks all publish to `/cmd_vel` — the driver does not care which.

![Wave Rover Robot](images/wave-rover-1.jpg)

---

## Architecture

Driver and teleop are separate processes:

```
Terminal 1 (RB3)                    Terminal 2 (RB3 or host PC)
┌────────────────────────────┐      ┌─────────────────────────┐
│ wave_rover_launch.py       │      │ joy_linux_node          │
│   → wave_rover_controller  │      │ teleop_twist_joy        │
│       UART → ESP32 motors  │      │   (loads teleop_joy.yaml)│
└─────────────┬──────────────┘      └────────────┬────────────┘
              │  /cmd_vel  ◄──────────────────────┘
              │
              │  /cmd_vel  ◄── nav2 / your stack (autonomy)
```

Run gamepad teleop **on whichever machine has the USB device** (RB3 or host PC).  
Use the same `ROS_DOMAIN_ID` so `/cmd_vel` reaches the robot.

---

## Quick start (RB3)

```bash
# Host: cross-compile and deploy (see Cross-compile section below)
cd WAVE-ROVER-Controller
./scripts/build.sh
./scripts/deploy.sh root@<rb3-ip>

# RB3: extract tarball, then:
mount -o remount,rw /usr
tar --no-overwrite-dir --no-same-owner -zxf /opt/wave_rover_controller.tar.gz -C /usr/
source /usr/share/qirp-setup.sh

# Terminal 1 — driver
ros2 launch wave_rover_controller wave_rover_launch.py
```

---

## Gamepad teleop

Start the driver first, then run the joy nodes in a **separate terminal**. Both nodes load settings from the same `teleop_joy.yaml`:

```bash
CONFIG=$(ros2 pkg prefix wave_rover_controller)/share/wave_rover_controller/config/teleop_joy.yaml
ros2 run joy_linux joy_linux_node --ros-args --params-file "$CONFIG"
ros2 run teleop_twist_joy teleop_node --ros-args --params-file "$CONFIG"
```

**All on one machine** (gamepad plugged into RB3):

```bash
# Terminal 1
ros2 launch wave_rover_controller wave_rover_launch.py

# Terminal 2
CONFIG=$(ros2 pkg prefix wave_rover_controller)/share/wave_rover_controller/config/teleop_joy.yaml
ros2 run joy_linux joy_linux_node --ros-args --params-file "$CONFIG"
ros2 run teleop_twist_joy teleop_node --ros-args --params-file "$CONFIG"
```

**Split** (driver on RB3, gamepad on host PC — same `ROS_DOMAIN_ID`):

```bash
# RB3 — driver only
ros2 launch wave_rover_controller wave_rover_launch.py

# Host PC — gamepad teleop (needs joy_linux + teleop_twist_joy installed)
CONFIG=$(ros2 pkg prefix wave_rover_controller)/share/wave_rover_controller/config/teleop_joy.yaml
ros2 run joy_linux joy_linux_node --ros-args --params-file "$CONFIG"
ros2 run teleop_twist_joy teleop_node --ros-args --params-file "$CONFIG"
```

> **Safety:** Ensure the robot is stationary before starting teleop.

> **Keyboard:** `config/teleop_keyboard.yaml` is available for manual `teleop_twist_keyboard` use (not bundled in deploy).

### Verify teleop config loaded

When `teleop_twist_joy` starts, it logs the active mapping. Compare against your `teleop_joy.yaml` — if you see package defaults (`enable_button` 5, `axis_linear.x` 5, `scale_linear.x` 0.5), the params file was not applied.

Example with current Microntek defaults in `teleop_joy.yaml`:

```
Teleop enable button 5.
Linear axis x on 1 at scale 0.300000.
Angular axis yaw on 0 at scale 1.000000.
```

The YAML keys must match the **ROS node names**, not the executable names:

| YAML section | Executable | ROS node name |
|--------------|--------------|---------------|
| `joy_node` | `joy_linux_node` | `joy_node` |
| `teleop_twist_joy_node` | `teleop_node` | `teleop_twist_joy_node` |

Use nested parameter groups (`axis_linear: { x: 1 }`) as in upstream `teleop_twist_joy` configs.

---

## Configuration

| File | What to tune |
|------|----------------|
| `config/wave_rover_controller.yaml` | UART, diff-drive (`wheel_separation`, `speed_scale`, motor limits) |
| `config/teleop_joy.yaml` | Gamepad device, axes, buttons, stick sensitivity |
| `config/teleop_keyboard.yaml` | Keyboard speed / turn rate |

On RB3: `/usr/share/wave_rover_controller/config/`  
Edit, then **restart** the affected node (no rebuild needed for YAML-only changes).

Driver startup log confirms config:

```
Config: UART=... speed_scale=... wheel_separation=...
```

---

## Cross-compile for RB3 Gen2 (QIR SDK)

Build bundles the driver plus `joy_linux` and `teleop_twist_joy` for the RB3.

```bash
cd <qirp-sdk>
source setup.sh

cd WAVE-ROVER-Controller
./scripts/build.sh
./scripts/deploy.sh root@<rb3-ip>
```

`build.sh` handles cross-compile hygiene automatically:

- Clears host ROS (`/opt/ros`) from prefix paths if present
- Unsets `LD_LIBRARY_PATH` (required by the QIR SDK)
- Uses the native x86 Python to run colcon
- Verifies all binaries are **aarch64**

If you prefer a clean shell, you can still run `unset LD_LIBRARY_PATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH` before `source setup.sh`.

For a full rebuild:

```bash
rm -rf build install log deps
./scripts/build.sh
```

See the QIR SDK `CROSS_COMPILE.md` in your SDK tree for the full workflow.

---

## ROS 2 topics

**Subscribed:** `/cmd_vel` (`geometry_msgs/msg/Twist`)

**Published:** `/cmd_vel_executed`, `/liveness`

---

## JSON protocol (UART)

```json
{"T": 1, "L": <left>, "R": <right>}
```

Motor command `T:1` with `L`/`R` in ±0.5 (firmware clamp). See source for OLED, WiFi, and e-stop commands.

---

## License

MIT
