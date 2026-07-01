# SmallBot — Auto Workspace

ROS 2 (Humble) workspace for autonomous robot:
**RPLidar A1M8** for SLAM mapping + **ESP32 SmallBot** controlling **2 × N20 encoder motors** via L298N driver.

## Directory Structure

```
auto/
├── src/
│   ├── slam_mapping/              ← RPLidar + SLAM Toolbox launch package
│   └── esp32_bridge/              ← ROS 2 USB serial bridge to ESP32
├── esp32_firmware/
│   └── esp32_motor_encoder/
│       └── esp32_motor_encoder.ino   ← Flash this to ESP32
├── launch/
│   └── full_system.launch.py      ← Master launch (SLAM + motors)
└── udev/
    └── 99-robot-usb.rules         ← USB port stability rules
```

---

## Quick Start

### 1. Build & source the workspace
```bash
cd ~/auto
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Flash ESP32
Open `esp32_firmware/esp32_motor_encoder/esp32_motor_encoder.ino` in **Arduino IDE**.
- Board: **ESP32 Dev Module**
- Upload speed: **921600**
- After flashing, open Serial Monitor at **115200 baud** — you should see:
  ```
  SMALLBOT PID READY (FIXED INTERRUPTS)
  E:0,0
  E:0,0
  ...
  ```

### 3. Set USB port permissions (one-time)
```bash
sudo usermod -aG dialout $USER   # then log out and back in
# Quick one-time fix (no logout needed):
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1
```

### 4. Install udev rules (locks USB port assignments)
```bash
sudo cp udev/99-robot-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 5. Run SLAM only (LiDAR mapping, no motors)
```bash
source ~/auto/install/setup.bash
ros2 launch slam_mapping slam.launch.py lidar_port:=/dev/ttyUSB0
```
RViz opens automatically showing the live map and laser scan.

### 6. Run ESP32 bridge only (motors + odometry, no SLAM)
```bash
source ~/auto/install/setup.bash
ros2 launch esp32_bridge esp32_bridge.launch.py esp32_port:=/dev/ttyUSB1
```
Terminal will print encoder ticks every second:
```
[TICKS]  L=  +1247  R=  +1261  ΔL=  +45  ΔR=  +46  vL=+0.174 m/s  vR=+0.171 m/s
```

### 7. Run full system (SLAM + ESP32 motors together)
```bash
source ~/auto/install/setup.bash
ros2 launch slam_mapping full_system.launch.py \
    lidar_port:=/dev/ttyUSB0 \
    esp32_port:=/dev/ttyUSB1
```

### 8. Drive the robot (new terminal)
```bash
source ~/auto/install/setup.bash
# Install teleop if not present:
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 9. Monitor encoder ticks live
```bash
# ROS odometry topic (computed position):
ros2 topic echo /odom --field pose.pose.position

# Disable tick printing if not needed:
ros2 launch esp32_bridge esp32_bridge.launch.py show_ticks:=false
```

---

## Hardware Wiring

### ESP32 → L298N Motor Driver

> [!NOTE]
> One L298N drives **both motors** (left = OUT1/OUT2, right = OUT3/OUT4).
> **Remove the ENA/ENB jumper caps** on the L298N board — those bypass speed control.
> Use **7–12 V** motor supply. L298N drops ~2 V internally.

| Signal | ESP32 GPIO | L298N Pin | Function |
|--------|-----------|-----------|----------|
| L_IN1  | **26** | IN1 | Left motor direction |
| L_IN2  | **27** | IN2 | Left motor direction |
| L_ENA  | **25** | **ENA** (PWM) | Left motor speed |
| R_IN3  | **32** | IN3 | Right motor direction |
| R_IN4  | **33** | IN4 | Right motor direction |
| R_ENB  | **4**  | **ENB** (PWM) | Right motor speed |

### ESP32 ← Encoders

| Signal | ESP32 GPIO | Pull-up | Notes |
|--------|-----------|---------|-------|
| Left Enc C1  (interrupt) | **34** | External 10kΩ to 3.3V | Input-only GPIO, no internal pull-up |
| Left Enc C2  (direction) | **35** | External 10kΩ to 3.3V | Input-only GPIO, no internal pull-up |
| Right Enc C1 (interrupt) | **18** | Internal `INPUT_PULLUP` | Standard GPIO |
| Right Enc C2 (direction) | **19** | Internal `INPUT_PULLUP` | Standard GPIO |

> [!WARNING]
> GPIO **34 and 35** on ESP32 are **input-only** with **NO internal pull-ups**.
> You MUST add external **10kΩ resistors** from pins 34 & 35 to **3.3V**, otherwise the left encoder will miscount or read random values.

### Status LED

| Signal | ESP32 GPIO | Behaviour |
|--------|-----------|-----------|
| STATUS_LED | **2** | Flashes on each received `C:` command |

---

## Serial Protocol

The ESP32 **PID firmware** handles wheel speed regulation internally.
ROS sends target velocities — not raw PWM values.

| Direction | Format | Example |
|-----------|--------|---------|
| **PC → ESP32** | `C:<v>,<omega>\n` | `C:0.2000,0.1500\n` |
| **ESP32 → PC** | `E:<left_ticks>,<right_ticks>\n` | `E:1247,1261\n` (20 Hz) |
| **Reset** | `R\n` | Resets encoder counts + PID integrators |

- `v` = linear velocity (m/s), `omega` = angular velocity (rad/s)
- `left_ticks` / `right_ticks` = **cumulative** encoder counts since boot (can go negative)

---

## Parameters

### esp32_bridge node

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `/dev/ttyUSB1` | ESP32 USB serial port |
| `baud` | `115200` | Baud rate |
| `wheel_diameter` | `0.043` | Wheel diameter in metres (43 mm N20 wheel) |
| `wheel_base` | `0.140` | Distance between left and right wheels (m) |
| `tpr_l` | `349.0` | Ticks/revolution — **LEFT** wheel (measured) |
| `tpr_r` | `362.0` | Ticks/revolution — **RIGHT** wheel (measured) |
| `publish_rate` | `20.0` | Odometry publish rate Hz (matches ESP32 loop) |
| `cmd_timeout` | `0.5` | Stop motors if no `/cmd_vel` for this many seconds |
| `show_ticks` | `true` | Print raw encoder ticks to terminal (1 Hz) |

### SLAM Toolbox

Edit `src/slam_mapping/config/slam_params.yaml`:
- `use_odometry: false` — pure LiDAR scan-matching (default, works without motors)
- `use_odometry: true` — fused SLAM using `/odom` from esp32_bridge
- `resolution: 0.05` — map cell size 5 cm

---

## Saving a Map

After building a map in RViz:
```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```
Saves `my_map.pgm` (image) and `my_map.yaml` (metadata) for Nav2 localization.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `Cannot open /dev/ttyUSB1` | Run `ls /dev/ttyUSB*`; adjust `esp32_port:=` arg |
| No encoder ticks (`E:0,0` always) | Check GPIO 34/35 pull-ups (10kΩ to 3.3V required) |
| Left encoder stuck / noisy | Add/replace external pull-up on GPIO 34 & 35 |
| Motors not moving | Send `C:0.2,0.0` in Serial Monitor; check L298N power & ENA/ENB |
| LiDAR not spinning | Check USB power; try `lidar_port:=/dev/ttyUSB0` |
| No map in RViz | Wait 5 s after launch; check `ros2 topic echo /scan` |
| Odom not updating | Check `ros2 topic echo /odom`; confirm ESP32 prints `E:` lines |
| Ports swapping on replug | Install udev rules (Step 4) |
