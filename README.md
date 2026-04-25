# CAMS-Bot

**CArry My Stuff Bot** — a self-balancing two-wheeled follower robot designed as assistive technology for individuals with mobility impairments. CAMS-Bot autonomously follows a person and carries up to 5&nbsp;kg of cargo in residential environments.

Developed for **EECE 5552 – Assistive Robotics** at Northeastern University.

---

## Repository layout

| Path | Contents |
| --- | --- |
| `hardware/` | CAD files and hardware documentation for 3D-printed and laser-cut parts |
| `pico/` | MicroPython firmware for the Raspberry Pi Pico (real-time balance controller) |
| `src/` | ROS 2 workspace source — packages including `cams_bringup` |
| `dockerfile` | Container definition for the ROS 2 environment on the Pi 4 |
| `.gitignore` | |

---

## Hardware

- **Compute:** Raspberry Pi 4 (high-level control, ROS 2) + Raspberry Pi Pico (real-time balance loop)
- **Sensing:** OAK-D Pro camera (vision / person tracking), MPU6050 IMU (balance)
- **Actuation:** DFRobot FIT0186 12&nbsp;V DC gearmotors with quadrature encoders, L298N dual H-bridge driver
- **Wheels:** 8-inch polyurethane (Grainger 416P25)
- **Chassis:** Three+ laser-cut acrylic decks; 3D-printed PLA brackets and mounts; PETG wheel-to-motor bushings

---

## Setup

You will need:

1. **A Raspberry Pi 4** running Ubuntu with Docker installed.
2. **A Raspberry Pi Pico** flashed with the MicroPython UF2 firmware. Download the latest UF2 from the [official MicroPython site](https://micropython.org/download/RPI_PICO/) and flash it by holding the BOOTSEL button while plugging the Pico into USB, then dragging the UF2 onto the mounted drive.
3. **An Xbox controller** for teleop, paired to the Pi 4 over Bluetooth.

### 1. Clone the workspace

```bash
git clone <this-repo> ~/cams_workspace
cd ~/cams_workspace
```

### 2. Flash the Pico balance controller

Copy `pico/balance_controller.py` onto the Pico **as `main.py`** so it runs automatically on boot. The recommended workflow is VS Code with the [MicroPico](https://marketplace.visualstudio.com/items?itemName=paulober.pico-w-go) extension.

### 3. Build the Docker image

From the workspace root (where the `dockerfile` lives):

```bash
sudo docker build -t cams -f dockerfile .
```

### 4. Pair the Xbox controller

Pair the Xbox controller to the Pi 4 over Bluetooth before launching. Once paired, it will appear under `/dev/input/`.

```bash
bluetoothctl
# > scan on
# > pair <MAC>
# > trust <MAC>
# > connect <MAC>
```

---

## Running

### Launch the container

```bash
sudo docker run -it \
  --name ros-cams \
  --network host \
  --device /dev/ttyAMA3 \
  --device /dev/input \
  -v ~/cams_workspace:/workspace \
  cams
```

The flags do the following:

- `--network host` — required for ROS 2 DDS discovery
- `--device /dev/ttyAMA3` — UART link to the Pico
- `--device /dev/input` — exposes the Xbox controller to the container
- `-v ~/cams_workspace:/workspace` — mounts the ROS 2 workspace into the container

### Build and launch CAMS-Bot

Inside the container:

```bash
cd /workspace
colcon build
source install/setup.bash
ros2 launch cams_bringup cams_bot.launch.py
```

### Teleop controls

| Input | Action |
| --- | --- |
| Left stick Y | Forward / backward |
| Right stick X | Turning |

---

## Architecture

```
Xbox Controller ──BT──> Pi 4 ──ROS 2──> [joystick → cmd_vel → serial bridge]
                                                                 │
                                                          /dev/ttyAMA3
                                                                 │
                                                                 ▼
                                                              Pico
                                                          (MicroPython
                                                       balance controller:
                                                         IMU + encoders →
                                                          PID → L298N)
                                                                 │
                                                                 ▼
                                                          DC gearmotors
```

The Pi 4 handles high-level control and teleop; the Pico runs the real-time PID balance loop at ~20&nbsp;kHz PWM. Communication between them uses a USB serial protocol (`D<float>,T<float>\n` for drive/turn, `S\n` for stop).

---

## Authors

Built by Jason Rupp and project partner for EECE 5552. Vision and person-tracking pipeline maintained separately.
