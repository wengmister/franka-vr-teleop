# VR Franka Robot Control

This system enables direct control of a Franka robot using Meta Quest 3 VR headset hand tracking. Your hand movements in VR are translated to robot end-effector pose commands with advanced smoothing, safety limits, and real-time visualization.

VR App repo @ [here](https://github.com/wengmister/quest-wrist-trackerw)


![arm vr con](https://github.com/user-attachments/assets/ccdd6b5d-a6ad-432a-9ac7-2af68a295281)




## Architecture

```
VR Headset → UDP → ROS2 Workstation → UDP → Realtime PC → Franka Robot
(hand tracking)  (pose data)  (pose processing)  (pose commands)  (libfranka)
```

## System Components

1. **VR Headset**: Streams right wrist tracking data via UDP (port 9000)
2. **ROS2 Workstation**: Processes VR data and converts to absolute pose commands (50Hz)
3. **Realtime PC**: Runs libfranka absolute pose control (1000Hz)
4. **Franka Robot**: Follows VR hand movements with sub-centimeter precision
5. **RViz2**: Real-time visualization of VR hand pose and robot target pose

## Key Features (Updated)

### Control Method:
- **Absolute Pose Control**: Robot maintains absolute position/orientation targets achieved through `libfranka` Cartesian Pose active control
- **Coordinate Transformation**: VR (unity's left-handed) coordinate to robot base frame coordinate transformation
- **Quaternion Smoothing**: SLERP-based orientation interpolation,
- **Real-time Visualization**: RViz2 integration for pose monitoring

### Safety & Stability:
- **Workspace Limits**: ±60cm from initial position
- **Automatic Error Recovery**: Manual confirmation before recovery attempts
- **Pose Validation**: Prevents invalid transformations
- **Network Resilience**: Handles UDP packet loss gracefully

## Setup Instructions

### 1. Realtime PC Setup (192.168.18.1)

#### Build Structure:
```
vr_robot_client/
├── CMakeLists.txt
├── examples_common.h      
├── src/
│   ├── examples_common.cpp
│   └── franka_vr_control_client.cpp
└── build/
```

#### Build Steps:
```bash
# Build in local PC
cd vr_robot_client/build
cmake .. # REMEMBER TO UPDATE LIBFRANKA INSTALL LOCATION
make

# Copy to real-time workstation
scp franka_vr_control_client YOUR_REALTIME_PC
```

### 2. ROS2 Workstation Setup (franka-vr-teleop)

#### Package Structure:
```
franka-vr-teleop/
├── src/franka_vr_teleop/
│   ├── franka_vr_teleop/
│   │   ├── __init__.py
│   │   └── vr_to_robot_converter.py
│   ├── launch/
│   │   └── vr_control.launch.py
│   ├── config/
│   │   └── pose_viz.rviz
│   ├── resource/
│   ├── test/
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
```

#### Setup Commands:
```bash
# Create workspace
mkdir -p ~/franka-vr-teleop
cd ~/franka-vr-teleop

# Copy package files to correct structure shown above

# Install dependencies
pip install numpy scipy

# Build ROS2 package
colcon build --packages-select franka_vr_teleop
source install/setup.bash
```

### 3. VR Headset Setup

VR App repo @ [here](https://github.com/wengmister/quest-wrist-trackerw)

The VR application should send UDP messages to **port 9000** in this format:
```
Right wrist:, x, y, z, qx, qy, qz, qw
```

Where:
- `x, y, z`: Position in meters (VR coordinate system)
- `qx, qy, qz, qw`: Orientation quaternion (VR coordinate system)

Example message:
```
Right wrist:, 0.123, 0.456, 0.789, 0.0, 0.0, 0.0, 1.0
```

## Usage (Updated for Absolute Pose Control)

### 1. Start Robot Controller (Realtime PC)

```bash
ssh user@192.168.18.1
cd vr_robot_client/build
./franka_vr_control_client 192.168.18.10
```

Expected output:
```
UDP server listening on port 8888 for VR pose data
WARNING: Robot will move based on VR input!
Put on your VR headset and make sure to have the emergency stop ready!
Press Enter to continue...
Finished moving to initial joint configuration.
Starting VR pose control. Move your VR hand to control robot.
VR reference pose initialized!
```

### 2. Start VR Processing with Visualization (ROS2 Workstation)

```bash
# Source environment
source /opt/ros/jazzy/setup.bash
source ~/franka-vr-teleop/install/setup.bash

# Launch VR control system with RViz2 visualization
ros2 launch franka_vr_teleop vr_control.launch.py
```

Expected output:
```
[INFO] [vr_to_robot_converter]: VR to Robot Converter started
[INFO] [vr_to_robot_converter]: VR UDP: 0.0.0.0:9000
[INFO] [vr_to_robot_converter]: Robot UDP: 192.168.18.1:8888
[INFO] [vr_to_robot_converter]: Move your VR hand to start control!
[INFO] [vr_to_robot_converter]: Initial VR pose captured!
[INFO] [vr_to_robot_converter]: VR input: 90.2 Hz | Command output: 50.0 Hz
```

**With custom parameters:**
```bash
ros2 launch franka_vr_teleop vr_control.launch.py \
    position_deadzone:=0.005 \
    orientation_deadzone:=0.02 \
    smoothing_factor:=0.05
```

### 3. Start VR Application

Just start the app ;)

### Available Launch Parameters:

```bash
ros2 launch franka_vr_teleop vr_control.launch.py \
    vr_udp_ip:='0.0.0.0' \              # VR data source IP
    vr_udp_port:=9000 \                 # VR data source port  
    robot_udp_ip:='192.168.18.1' \      # Robot PC IP
    robot_udp_port:=8888 \              # Robot PC port
    position_deadzone:=0.01 \           # Position sensitivity (meters)
    orientation_deadzone:=0.01 \        # Orientation sensitivity (radians)
    smoothing_factor:=0.1               # Smoothing factor (0=no smoothing, 1=max smoothing)
```

### Tuning Examples:

**High Precision** (for detailed work):
```bash
ros2 launch franka_vr_teleop vr_control.launch.py \
    position_deadzone:=0.003 \
    orientation_deadzone:=0.005 \
    smoothing_factor:=0.05
```

**Stable Operation** (for noisy VR tracking):
```bash
ros2 launch franka_vr_teleop vr_control.launch.py \
    position_deadzone:=0.02 \
    orientation_deadzone:=0.03 \
    smoothing_factor:=0.2
```

# Demo

![hand vr con](https://github.com/user-attachments/assets/b7df9406-0af2-440f-aec6-61f3f7d252d9)

Open-source robot hand @ [this repo](https://github.com/wengmister/BiDexHand)


# License
MIT
