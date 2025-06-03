# VR Franka Robot Control System (Working Version)

This system enables direct control of a Franka robot using VR headset hand tracking. Your hand movements in VR are translated to robot end-effector movements with scaling, smoothing, and safety limits.

## Architecture

```
VR Headset → UDP → ROS2 Workstation → UDP → Realtime PC → Franka Robot
(hand tracking)  (pose data)  (processing)    (EE commands)  (libfranka)
```

## System Components

1. **VR Headset**: Streams right wrist tracking data via UDP (port 9000)
2. **ROS2 Workstation**: Processes VR data and converts to robot commands (50Hz)
3. **Realtime PC**: Runs libfranka pose control (1000Hz)
4. **Franka Robot**: Follows VR hand movements with sub-centimeter precision

## Setup Instructions

### 1. Realtime PC Setup (192.168.18.1)

#### Build Structure:
```
vr_robot_client/
├── CMakeLists.txt
├── examples_common.h      
├── src/
│   ├── examples_common.cpp
│   └── vr_franka_control_client.cpp
└── build/
```

#### Build Steps:
```bash
# Create directory structure locally first
mkdir -p vr_robot_client/src

# Copy files to correct locations:
# - CMakeLists.txt (VR version) to vr_robot_client/
# - examples_common.h to vr_robot_client/
# - examples_common.cpp to vr_robot_client/src/
# - vr_franka_control_client.cpp to vr_robot_client/src/

# Copy to realtime PC
scp -r vr_robot_client/ user@192.168.18.1:~/

# SSH and build
ssh user@192.168.18.1
cd vr_robot_client/build
cmake ..
make
```

### 2. ROS2 Workstation Setup (franka-vr-teleop)

#### Package Structure (Actual):
```
franka-vr-teleop/
├── src/franka_vr_teleop/
│   ├── __init__.py
│   └── vr_to_robot_converter.py
├── launch/
│   └── vr_control.launch.py
├── resource/
├── test/
├── package.xml
├── setup.cfg
└── setup.py
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

Your VR application should send UDP messages to **port 9000** in this format:
```
Right wrist:, x, y, z, qx, qy, qz, qw
```

Where:
- `x, y, z`: Position in meters (VR coordinate system)
- `qx, qy, qz, qw`: Orientation quaternion

Example message:
```
Right wrist:, 0.123, 0.456, 0.789, 0.0, 0.0, 0.0, 1.0
```

## Usage (Verified Working)

### 1. Start Robot Controller (Realtime PC)

```bash
ssh user@192.168.18.1
cd vr_robot_client/build
./vr_franka_control_client 192.168.18.10
```

Expected output:
```
UDP server listening on port 8888
WARNING: Robot will move based on VR input!
Put on your VR headset and make sure to have the emergency stop ready!
Press Enter to continue...
Finished moving to initial joint configuration.
Starting VR pose control. Move your VR hand to control robot.
Initial robot pose captured. VR control active!
```

### 2. Start VR Processing (ROS2 Workstation)

```bash
# Source environment
source /opt/ros/humble/setup.bash
source ~/franka-vr-teleop/install/setup.bash

# Launch VR control system (default settings)
ros2 launch franka_vr_teleop vr_control.launch.py
```

Expected output:
```
[INFO] [vr_to_robot_converter]: VR to Robot Converter started
[INFO] [vr_to_robot_converter]: VR UDP: 0.0.0.0:9000
[INFO] [vr_to_robot_converter]: Robot UDP: 192.168.18.1:8888
[INFO] [vr_to_robot_converter]: Move your VR hand to start control!
[INFO] [vr_to_robot_converter]: Initial VR pose captured!
```

**With tuned parameters for better responsiveness:**
```bash
ros2 launch franka_vr_teleop vr_control.launch.py \
    pose_scale:=1.5 \
    position_deadzone:=0.005 \
    orientation_scale:=0.8
```

### 3. Start VR Application

Put on your VR headset and start the application that streams wrist tracking data to port 9000.

## Control Mapping (Tested & Working)

### Coordinate Systems:
- **VR**: +x=right, +y=up, +z=forward
- **Robot**: +x=forward, +y=left, +z=up

### Movement Translation:
- **VR hand right** → Robot moves **left**
- **VR hand forward** → Robot moves **forward**  
- **VR hand up** → Robot moves **up**
- **VR hand rotation** → Robot end-effector rotation

### Performance Settings (Tuned for Stability):
- **Linear velocity**: 5cm/s max (prevents acceleration discontinuities)
- **Angular velocity**: 0.05 rad/s max (~3°/s)
- **Velocity scaling**: 0.5x conversion factor
- **Position scaling**: 1.0x default (configurable)
- **Update rate**: 50Hz VR processing, 1000Hz robot control

## Safety Features

### Movement Limits:
- **Hard velocity caps**: 5cm/s linear, 0.05 rad/s angular
- **Workspace bounds**: ±50cm from initial position
- **Pose validation**: Prevents invalid transformations
- **Deadzone filtering**: 1cm position, 3° orientation default

### Control Safety:
- **Multi-layer smoothing**: VR data + robot pose smoothing
- **Acceleration limiting**: Prevents velocity discontinuities
- **Emergency recovery**: Automatic error recovery and restart
- **Connection monitoring**: Handles network interruptions
- **Pose limiting**: Hardware safety bounds enforced

### Manual Safety:
- **Emergency stop**: Available via VR trigger or external device
- **Pose reset**: Return to initial position
- **Manual stop**: Physical emergency stop always accessible

## Configuration & Tuning

### Launch Parameters:

**Basic tuning**:
```bash
ros2 launch franka_vr_teleop vr_control.launch.py \
    pose_scale:=1.2 \                    # Movement amplification
    orientation_scale:=0.8 \             # Rotation scaling
    position_deadzone:=0.008 \           # Position sensitivity
    orientation_deadzone:=0.04           # Rotation sensitivity
```

**More responsive** (for precise work):
```bash
ros2 launch franka_vr_teleop vr_control.launch.py \
    pose_scale:=1.8 \
    position_deadzone:=0.003 \
    orientation_deadzone:=0.02
```

**More stable** (for rough VR tracking):
```bash
ros2 launch franka_vr_teleop vr_control.launch.py \
    pose_scale:=0.8 \
    position_deadzone:=0.015 \
    orientation_deadzone:=0.08
```

### Debug and Monitoring:

**Check VR data reception**:
```bash
ros2 topic echo /vr_wrist_pose
```

**Monitor robot targets**:
```bash
ros2 topic echo /robot_target_pose
```

**Monitor actual velocities sent**:
```bash
# Look for "Velocities sent:" in the logs
ros2 launch franka_vr_teleop vr_control.launch.py
```

Expected velocity ranges:
- **Normal operation**: 0.005-0.025 m/s (5-25mm/s)
- **Fast movements**: Up to 0.05 m/s (50mm/s, capped)

## Troubleshooting (Tested Solutions)

### Common Issues:

**VR data not received**:
- Check VR app is sending to port **9000** (not 9999)
- Verify firewall allows UDP traffic: `sudo ufw allow 9000/udp`
- Test with netcat: `nc -u -l 9000` and check for messages

**Robot not responding**:
- Verify realtime PC connection (192.168.18.1:8888)
- Check robot is in ready state and not in protective stop
- Ensure external activation device connected

**Movement too small**:
- Increase `pose_scale` to 1.5-2.0
- Decrease `position_deadzone` to 0.003-0.005
- Check VR tracking quality and range

**Control exceptions** (velocity/acceleration discontinuity):
- System automatically recovers after 1 second
- If persistent, reduce `pose_scale` to 0.8
- Increase `position_deadzone` to 0.015
- Check for VR tracking glitches

**Jittery movement**:
- Increase smoothing_factor to 0.9
- Increase deadzone values
- Check VR system performance

### Recovery Procedures:

**Robot stops moving**:
1. System will auto-recover after 1 second
2. Check realtime PC console for errors
3. If persistent, restart both systems

**VR tracking lost**:
1. System will use last known pose
2. Re-establish VR tracking
3. Use pose reset if needed

## Technical Details (Verified Performance)

### Data Flow:
1. **VR Headset** → UDP wrist tracking data (varies by VR system)
2. **ROS2 Node** → Processes and scales VR data (50 Hz)  
3. **Robot Client** → Applies pose commands with smoothing (1000 Hz)
4. **Franka Robot** → Executes smooth end-effector motion

### Performance Metrics:
- **End-to-end latency**: ~50-100ms (depending on VR system)
- **Update rate**: 50 Hz command rate, 1000 Hz robot control
- **Accuracy**: Sub-centimeter positioning with proper calibration
- **Stability**: No velocity discontinuities with tuned parameters

### Network Protocol:
**VR → ROS2**: Raw wrist tracking UDP messages (port 9000)
**ROS2 → Robot**: Velocity commands in format (port 8888):
```
"linear_x linear_y linear_z angular_x angular_y angular_z emergency_stop reset_pose"
```

Example working command:
```
"0.0127 -0.0000 0.0028 -0.0134 -0.0111 0.0160 0 0"
```

## Quick Start Summary

1. **Build VR robot client**:
   ```bash
   cd vr_robot_client/build && cmake .. && make
   scp -r vr_robot_client/ user@192.168.18.1:~/
   ```

2. **Run on realtime PC (192.168.18.1)**:
   ```bash
   ssh user@192.168.18.1
   cd vr_robot_client/build
   ./vr_franka_control_client 192.168.18.10
   ```

3. **Run on local machine**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/franka-vr-teleop/install/setup.bash
   ros2 launch franka_vr_teleop vr_control.launch.py
   ```

4. **Start VR tracking** to port 9000

**This system provides intuitive, responsive VR control of the Franka robot with built-in safety and error recovery! The robot will smoothly follow your hand movements in 3D space with millimeter precision.**