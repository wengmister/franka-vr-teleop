# Franka VR Robot Teleoperation

This system enables high-performance VR teleoperation of a Franka robot using advanced trajectory generation and inverse kinematics. Your hand movements in VR are translated to smooth, responsive robot motion through joint-space velocity control with jerk-limited trajectories.

Meta Quest VR App @ [this repo](https://github.com/wengmister/quest-wrist-tracker)

<a href="https://www.youtube.com/embed/zSQQ5LxgFGo?si=zg_xzxWG-oeq02WC" target="_blank">
  <img src="https://img.youtube.com/vi/zSQQ5LxgFGo/maxresdefault.jpg" alt="Demo Video" width="560">
</a>

## Changelog

<details>
<summary>Click to expand changelog</summary>

v1.2:
- Added `pause` mode:
  - Allows user to pause and reposition while left fist is clenched.
  - Output commmand will restart from the paused position upon fist release.
  - added launch arg `pause_enabled = false`. When set to `true`, enabls pause mode.
- `pause` mode requires updated app under `dex-retargeter` branch on the vr repo.
- **important** - VR wolrd frame has an origin, when crossing the origin while app is open will cause the coordinates to be mirrored. Remember to reset world frame by holding down meta menu (pinching thumb and index while the menu button is invoked)

v1.1:
- Updated IK algorithm with Brent's method based 1d-optimization
  - Score weights can be adjusted to balance the priority in optimization for the following attributes
    - Yoshikawa manipulability
    - distance from current pose (very important for joint trajectory continuity)
    - distance from neutral pose

</details>

## Architecture

```
VR Headset → UDP → ROS2 Workstation → UDP → VR Robot Client (Realtime PC) → libfranka → Franka Robot
(hand tracking)     (pose data)      (IK+Ruckig trajectory generation)     (joint velocity control)
```

## System Components

1. **VR Headset**: Streams hand tracking data via UDP (port 8888)
2. **ROS2 Node**: Wrist vector visualization, frame conversion, and input data smoothing.
3. **VR Robot Client**: Real-time system with kinematic motion control:
   - **Weighted IK**: Optimizes joint configurations for manipulability, smoothness, and base stability. This is an implementation adapted from [PC Lopez-Custodio et al.'s amazing work, GeoFIK](https://github.com/PabloLopezCustodio/GeoFIK).
   - **Ruckig Trajectory Generator**: Provides jerk-limited, time-optimal motion profiles 
   - **Joint-Space Velocity Control**: Direct velocity commands for responsive control
   - **Real-time Processing**: 1kHz control loop with <1ms trajectory calculations
4. **Franka Robot**: Motion profile output and controlled via `libfranka`

## Setup Instructions

### Prerequisites

- **libfranka**: For Franka robot control. You will need a version corresponding to your firmware version.
- **Ruckig**: For trajectory generation (`sudo apt install libruckig-dev` or build from [source](https://github.com/pantor/ruckig))
- **Eigen3**: For linear algebra (`sudo apt install libeigen3-dev`)
- **ROS2**: For frame conversion, data smoothing and wrist vector visualization.

### VR Robot Client Setup

#### Structure:
```
vr_robot_client/
├── CMakeLists.txt
├── include/
│   ├── examples_common.h
│   ├── geofik.h           # Geometric inverse kinematics
│   └── weighted_ik.h      # Weighted IK solver with optimization and stabilization
├── src/
│   ├── examples_common.cpp
│   ├── geofik.cpp         # Franka analytical IK
│   ├── weighted_ik.cpp    # Multi-criteria IK optimization
│   └── franka_vr_control_client.cpp  # Main VR control system
└── build/
```

#### Build:
```bash
mkdir vr_robot_client/build && cd vr_robot_client/build
cmake -DFRANKA_INSTALL_PATH=/your/path/to/libfranka/install ..
make -j4

# Run the VR control client on your realtime workstation
./franka_vr_control_client <robot-hostname>
```

#### Notes:
- **libfranka**: Real-time robot control interface
- **Ruckig**: Time-optimal trajectory generation with jerk constraints
- **geofik**: Custom geometric inverse kinematics library for Franka
- **weighted_ik**: Multi-objective IK solver optimizing manipulability, joint limits, and base stability

### ROS2 Node:

```bash

rosdep update && rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-select franka_vr_teleop
```

### VR Headset Setup

Meta Quest VR App @ [this repo](https://github.com/wengmister/quest-wrist-tracker)

The VR application should send UDP messages to **port 9000** with:
```
x y z qx qy qz qw
```

Where:
- `x, y, z`: Position in meters (VR coordinate system)  
- `qx, qy, qz, qw`: Orientation quaternion (x, y, z, w format)

Example message:
```
0.123 0.456 0.789 0.0 0.0 0.0 1.0
```

## Usage

### 1. Start VR Robot Client

```bash
cd vr_robot_client/build
./franka_vr_control_client <robot-hostname> [bidexhand]
```

When [bidexhand] is set to `true`, IK solver will limit the joint range of J7 to prevent damaging the servo sleeve attachment. Argument currently defaults to true.

### 2. Start VR Application

Put on your VR headset and start the application `arm tracker` that streams hand tracking data to the ROS2 node.

### 3. Launch ROS2 Node

```bash
# on ROS2 workstation
. install/setup.bash
ros2 launch franka_vr_teleop vr_control.launch.py
```

The robot teleop will be live now!

## Demo

![tele_1](https://github.com/user-attachments/assets/9d8e6402-52b3-4378-9186-89616f87d592)


