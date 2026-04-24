# Autonomous Drone Landing on a Moving Platform

A ROS 2 + Gazebo simulation of a quadrotor UAV that autonomously takes off, locates a moving ground platform via an ArUco fiducial marker, and lands on it — even while the platform is translating and bobbing in all three axes.

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [ROS 2 Nodes](#ros-2-nodes)
- [State Machine](#state-machine)
- [Platform Motion](#platform-motion)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Running the Simulation](#running-the-simulation)
- [ROS 2 Topics](#ros-2-topics)
- [Configuration](#configuration)
- [Project Structure](#project-structure)

---

## Overview

The drone starts at ground level, takes off to a cruise altitude of 4 m, searches for the landing platform, aligns itself using computer-vision-based ArUco pose estimation fused with odometry-based feedforward, and descends onto the moving target. Once landed, the drone tracks the platform's velocity in all three axes so it stays on board as the platform continues to move.

Key features:

- **ArUco marker detection** — OpenCV detects a 4×4 ArUco marker (ID 0) from a downward-facing camera and solves for its 6-DoF pose with `solvePnP`.
- **PID + feedforward control** — Separate PID loops for X, Y, and Z with velocity feedforward from a least-squares velocity estimator to compensate for platform motion.
- **3-D moving target** — The platform executes configurable sine/cosine/Lissajous patterns in XY while bobbing vertically, making the landing challenging and realistic.
- **Finite-state machine** — Clean state transitions (IDLE → TAKEOFF → SEARCH → APPROACH → DESCEND → LANDED) with automatic recovery.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                        Gazebo                           │
│   ┌──────────────┐          ┌──────────────────────┐    │
│   │  Quadrotor   │  /drone/ │  ArUco Platform      │    │
│   │  (planar     │  odom    │  (planar move plugin) │    │
│   │   move       │          └──────────────────────┘    │
│   │   plugin)    │                    │ /platform/odom   │
│   │  + camera    │                    │                  │
│   └──────────────┘                    │                  │
│         │ image_raw / camera_info     │                  │
└─────────┼─────────────────────────────┼──────────────────┘
          │                             │
          ▼                             │
  ┌───────────────┐                     │
  │ aruco_detector│──/landing/          │
  │  (OpenCV)     │  target_pose        │
  └───────────────┘  marker_detected    │
                            │           │
                            ▼           ▼
                   ┌─────────────────────────┐
                   │   landing_controller    │
                   │   (PID + feedforward)   │──► /drone/cmd_vel
                   └─────────────────────────┘
                            │ /landing/status
                            │ /landing/errors
                            ▼
                   ┌─────────────────────────┐
                   │     state_machine       │──► /landing/state_command
                   └─────────────────────────┘
                            │
                   ┌─────────────────────────┐
                   │  platform_controller    │──► /platform/cmd_vel
                   └─────────────────────────┘
```

---

## ROS 2 Nodes

| Node | Executable | Description |
|------|-----------|-------------|
| `aruco_detector` | `aruco_detector` | Subscribes to the downward camera, detects ArUco marker ID 0, and publishes 6-DoF pose and a detection flag. Supports both the OpenCV ≥ 4.7 and legacy ArUco APIs. |
| `landing_controller` | `landing_controller` | Runs PID control loops for X, Y, Z. Applies velocity feedforward from a sliding-window least-squares estimator. Transitions to "ride" mode after landing. |
| `state_machine` | `state_machine` | Orchestrates high-level mission states. Handles timeouts and recovery (e.g., lost platform → back to SEARCH). |
| `platform_controller` | `platform_controller` | Drives the landing platform along a parameterised trajectory (sine, circle, or Lissajous) with independent Z bobbing. |

---

## State Machine

```
IDLE ──► TAKEOFF ──► SEARCH ──► APPROACH ──► DESCEND ──► LANDED
                        ▲            │              │
                        └────────────┘◄─────────────┘
                          (recovery)     (drift recovery)
```

| State | Entry condition | Action |
|-------|----------------|--------|
| `IDLE` | Initial | No motion |
| `TAKEOFF` | Mission start (auto, 2 s delay) | Climb at 0.8 m/s until 4 m altitude |
| `SEARCH` | Takeoff done | Fly toward platform odometry position |
| `APPROACH` | Platform within 3 m | PID + feedforward alignment at hover altitude |
| `DESCEND` | XY error < 0.8 m | Slow descent at 0.15 m/s, holds if drift detected |
| `LANDED` | Drone Z < 0.5 m and XY < 0.65 m | Match platform velocity in all axes |

---

## Platform Motion

The platform is controlled by `platform_controller` and moves according to the `pattern` parameter:

| Pattern | X velocity | Y velocity |
|---------|-----------|-----------|
| `sine` (default) | A·ω·cos(ωt) | 0.6·A·ω·cos(ωt + π/3) |
| `circle` | −A·ω·sin(ωt) | A·ω·cos(ωt) |
| `lissajous` | A·ω·cos(ωt) | A·ω·cos(2ωt) |

Z bobbing is always active: `Z_amp · 0.5ω · cos(0.5ωt)`.

Default parameters: amplitude = 1.5 m, Z amplitude = 0.3 m, frequency = 0.15 Hz.

---

## Prerequisites

| Dependency | Version |
|-----------|---------|
| Ubuntu | 22.04 |
| ROS 2 | Humble Hawksbill |
| Gazebo | Fortress / Classic (gazebo_ros_pkgs) |
| Python | ≥ 3.10 |
| OpenCV | ≥ 4.5 (with `opencv-contrib-python` for ArUco) |
| NumPy | any recent |

Install ROS 2 dependencies:

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-xacro \
  python3-opencv \
  python3-numpy
```

---

## Installation

```bash
# 1. Clone the repository
git clone https://github.com/Samarth-2705/Mobile-Autonomous-Robotics-Mini-Project.git
cd Mobile-Autonomous-Robotics-Mini-Project/drone_ws

# 2. Generate the ArUco marker image, Gazebo model, and material script
python3 generate_marker.py

# 3. Source the setup file produced by generate_marker.py
source ~/.bashrc

# 4. Install ROS 2 dependencies
rosdep install --from-paths src --ignore-src -r -y

# 5. Build
colcon build --symlink-install

# 6. Source the workspace
source install/setup.bash
```

---

## Running the Simulation

```bash
# Launch Gazebo, RViz, and all nodes with a single command
ros2 launch drone_landing simulation.launch.py
```

The launch file starts nodes in a timed sequence to let Gazebo fully load before spawning models:

| Time (s) | Action |
|---------|--------|
| 0 | Gazebo, RViz, state publishers |
| 7 | Spawn drone at (0, 0, 4.5) |
| 11 | Spawn platform at (2, 0, 0.025) |
| 14 | Platform controller |
| 16 | ArUco detector |
| 18 | Landing controller |
| 20 | State machine (mission auto-starts 2 s later) |

A successful landing prints:

```
╔══════════════════════════════════════════════╗
║          SUCCESSFULLY LANDED !               ║
║  Center error   :   X.X cm                  ║
║  Drone altitude : X.XXX m                   ║
║  Platform Z     : X.XXX m                   ║
╚══════════════════════════════════════════════╝
```

---

## ROS 2 Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/drone/cmd_vel` | `geometry_msgs/Twist` | `landing_controller`, `state_machine` | Velocity commands to drone |
| `/drone/odom` | `nav_msgs/Odometry` | Gazebo plugin | Drone pose and velocity |
| `/drone/downward_camera/image_raw` | `sensor_msgs/Image` | Gazebo camera | Raw camera frames |
| `/drone/downward_camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo camera | Camera intrinsics |
| `/platform/cmd_vel` | `geometry_msgs/Twist` | `platform_controller` | Velocity commands to platform |
| `/platform/odom` | `nav_msgs/Odometry` | Gazebo plugin | Platform pose and velocity |
| `/landing/target_pose` | `geometry_msgs/PoseStamped` | `aruco_detector` | Marker pose in camera frame |
| `/landing/marker_detected` | `std_msgs/Bool` | `aruco_detector` | Detection flag |
| `/landing/debug_image` | `sensor_msgs/Image` | `aruco_detector` | Annotated camera frame |
| `/landing/state_command` | `std_msgs/String` | `state_machine` | Current state broadcast |
| `/landing/status` | `std_msgs/String` | `landing_controller` | Landing status (e.g. `LANDED`) |
| `/landing/errors` | `std_msgs/Float32MultiArray` | `landing_controller` | `[err_x, err_y, err_z, xy_err, vx_plat, vy_plat, actual_xy, z_gap]` |

---

## Configuration

Platform motion can be tuned at launch time:

```bash
ros2 launch drone_landing simulation.launch.py
# Edit launch/simulation.launch.py → platform_ctrl parameters:
#   pattern:    'sine' | 'circle' | 'lissajous'
#   amplitude:   1.5   (metres, XY)
#   z_amplitude: 0.3   (metres, Z bob)
#   frequency:   0.15  (Hz)
```

Key `landing_controller` parameters (edit source):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `approach_alt` | 4.0 m | Cruise altitude |
| `hover_above` | 0.8 m | Height above platform during approach |
| `descent_rate` | 0.15 m/s | Speed of final descent |
| `align_thresh` | 0.40 m | XY error below which descent begins |
| `landed_z_thresh` | 0.60 m | Z threshold for landing detection |
| `landed_xy_thresh` | 0.65 m | XY threshold for landing detection |

---

## Project Structure

```
Mobile-Autonomous-Robotics-Mini-Project/
└── drone_ws/
    ├── generate_marker.py          # One-time setup: generates ArUco marker & Gazebo model
    └── src/
        └── drone_landing/
            ├── drone_landing/
            │   ├── aruco_detector.py       # OpenCV ArUco detection + PnP pose estimation
            │   ├── landing_controller.py   # PID + feedforward velocity controller
            │   ├── platform_controller.py  # Sinusoidal platform motion driver
            │   └── state_machine.py        # High-level mission state machine
            ├── launch/
            │   └── simulation.launch.py    # Full simulation launch file
            ├── models/
            │   └── aruco_platform/         # Generated Gazebo model (ArUco textured platform)
            ├── urdf/
            │   ├── drone.urdf.xacro        # Quadrotor URDF with camera + IMU plugins
            │   └── platform.urdf.xacro     # Landing platform URDF
            ├── worlds/
            │   └── landing_world.world     # Gazebo world file
            ├── rviz/
            │   └── drone_landing.rviz      # RViz configuration
            ├── package.xml
            └── setup.py
```
