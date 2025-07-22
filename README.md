# VR Franka Robot Teleoperation System

This system enables high-performance VR teleoperation of a Franka robot using advanced trajectory generation and inverse kinematics. Your hand movements in VR are translated to smooth, responsive robot motion through joint-space velocity control with jerk-limited trajectories.

## Architecture

```
VR Headset → UDP → VR Robot Client (Realtime PC) → libfranka → Franka Robot
(hand tracking)  (pose data)  (IK+Ruckig trajectory generation)  (velocity control)
```

## System Components

1. **VR Headset**: Streams hand tracking data via UDP (port 8888)
2. **VR Robot Client**: Real-time system with advanced motion control:
   - **Weighted IK**: Optimizes joint configurations for manipulability and smoothness. This is a custom implementation adopted from [PC Lopez-Custodio et al.'s work, GeoFIK](https://github.com/PabloLopezCustodio/GeoFIK). (This is an amazing analytical solver for Franka)
   - **Ruckig Trajectory Generator**: Provides jerk-limited, time-optimal motion profiles  
   - **Joint-Space Velocity Control**: Direct velocity commands for responsive control
   - **Real-time Processing**: 1kHz control loop with <1ms trajectory calculations
3. **Franka Robot**: Executes smooth, responsive motion without reflexes or discontinuities

## Setup Instructions

### Prerequisites

- **libfranka**: For Franka robot control. You will need a version corresponding to your firmware version.
- **Ruckig**: For trajectory generation (`sudo apt install libruckig-dev` or build from [source](https://github.com/pantor/ruckig))
- **Eigen3**: For linear algebra (`sudo apt install libeigen3-dev`)

### VR Robot Client Setup

#### Project Structure:
```
vr_robot_client/
├── CMakeLists.txt
├── include/
│   ├── examples_common.h
│   ├── geofik.h           # Geometric inverse kinematics
│   └── weighted_ik.h      # Weighted IK solver
├── src/
│   ├── examples_common.cpp
│   ├── geofik.cpp         # Franka kinematic functions
│   ├── weighted_ik.cpp    # Multi-criteria IK optimization
│   └── franka_vr_control_client.cpp  # Main VR control system
└── build/
```

#### Build Steps:
```bash
cd vr_robot_client/build
cmake ..
make -j4

# Run the VR control client
./franka_vr_control_client <robot-hostname>
```

#### Key Dependencies:
- **libfranka**: Real-time robot control interface
- **Ruckig**: Time-optimal trajectory generation with jerk constraints
- **geofik**: Custom geometric inverse kinematics library for Franka
- **weighted_ik**: Multi-objective IK solver optimizing manipulability and joint limits

### VR Headset Setup

Your VR application should send UDP messages to **port 8888** (directly to the robot client) in this format:
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

**Note**: This system now uses direct VR→Robot communication, eliminating the ROS2 workstation middleman for reduced latency and improved performance.

## Usage

### 1. Start VR Robot Client

```bash
cd vr_robot_client/build
./franka_vr_control_client <robot-hostname>
```

Expected output:
```
UDP server listening on port 8888 for VR pose data
WARNING: This example will move the robot! 
Please make sure to have the user stop button at hand!
Press Enter to continue...

Finished moving to initial joint configuration.
Ruckig trajectory generator configured with 7 DOFs
Waiting for VR data...
VR reference pose initialized!
VR initialized! Starting real-time control.
Ruckig initialized for velocity control!
Starting with zero velocity commands to smoothly take over control
```

### 2. Start VR Application

Put on your VR headset and start the application that streams hand tracking data to **port 8888** on the robot client.

The system will:
1. **Initialize smoothly** with zero velocity commands
2. **Gradually activate** over 0.5 seconds for safety
3. **Provide responsive control** with minimal latency
4. **Generate smooth trajectories** using Ruckig's jerk-limited profiles

## Advanced Control Features

### Weighted Inverse Kinematics
The system uses a multi-objective IK solver that optimizes:
- **Manipulability**: Avoids singular configurations for better control
- **Neutral Pose Distance**: Keeps joints close to comfortable positions  
- **Current Pose Distance**: Minimizes joint motion for smooth transitions

### Trajectory Generation (Ruckig)
- **Time-optimal trajectories**: Fastest motion within kinematic constraints
- **Jerk-limited profiles**: Smooth acceleration changes prevent robot stress
- **Real-time capable**: <1ms computation time for 1kHz control
- **Velocity control**: Direct joint velocity commands for responsive motion

### Performance Settings (Tuned for Responsiveness):
- **Joint Velocities**: 1.7-2.0 rad/s max (responsive motion)
- **Joint Accelerations**: 4.0-6.0 rad/s² (snappy response)
- **Joint Jerk**: 8.0-12.0 rad/s³ (smooth but quick transitions)
- **VR Smoothing**: 0.05 (minimal filtering for responsiveness)
- **Activation Time**: 0.5s gradual ramp-up for safety
- **Update Rate**: 1kHz trajectory generation and robot control

## Safety Features

### Built-in Safety Systems:
- **Joint Limit Enforcement**: Q7 clamped to [-0.2, 1.9] rad for safe operation with [BiDexHand](https://github.com/wengmister/BiDexHand)
- **Workspace Limits**: 0.75m maximum offset from initial position
- **Velocity Limits**: Configurable per-joint velocity constraints
- **Jerk Limiting**: Ruckig prevents harmful acceleration changes
- **Gradual Activation**: 0.5s ramp-up prevents sudden movements

### Robust Error Handling:
- **IK Fallbacks**: System continues with previous targets if IK fails
- **Trajectory Fallbacks**: Zero velocity commands on Ruckig errors  
- **Connection Monitoring**: Handles VR data loss gracefully
- **Reflex Prevention**: Joint velocity control eliminates motion discontinuities
- **Automatic Recovery**: System designed to avoid protective stops

### Physical Safety:
- **Emergency Stop**: Physical hardware emergency stop always active
- **Collision Behavior**: Conservative collision detection settings
- **Joint Impedance**: Moderate impedance for safe human interaction
- **Manual Override**: Physical intervention always possible

## Configuration & Tuning

### Key Parameters (in source code):

**Responsiveness Tuning** (`franka_vr_control_client.cpp`):
```cpp
// VR input filtering
double vr_smoothing = 0.05;              // Lower = more responsive (0.02-0.1)
double position_deadzone = 0.001;        // Smaller = more sensitive (0.0005-0.005) 
double orientation_deadzone = 0.03;      // Smaller = more sensitive (0.01-0.05)

// Activation timing
double ACTIVATION_TIME_SEC = 0.5;        // Faster startup (0.1-2.0s)

// Joint motion limits
MAX_JOINT_VELOCITY = {1.7, 1.7, 1.7, 1.7, 2.0, 2.0, 2.0};      // Higher = faster
MAX_JOINT_ACCELERATION = {4.0, 4.0, 4.0, 4.0, 6.0, 6.0, 6.0};  // Higher = snappier  
MAX_JOINT_JERK = {8.0, 8.0, 8.0, 8.0, 12.0, 12.0, 12.0};       // Higher = more responsive
```

**Tuning Guidelines**:
- **More Responsive**: Decrease `vr_smoothing` to 0.02, increase accelerations to 6.0-8.0
- **More Stable**: Increase `vr_smoothing` to 0.1, decrease accelerations to 2.0-3.0  
- **Precise Control**: Decrease deadzones to 0.0005/0.01, reduce `ACTIVATION_TIME_SEC` to 0.1
- **Safer Operation**: Increase deadzones to 0.005/0.05, increase `ACTIVATION_TIME_SEC` to 1.0

### Debug and Monitoring:

**Check VR data reception**:
```bash
# Monitor UDP traffic on port 8888
sudo netstat -ulnp | grep 8888

# Test VR connection manually
echo "0.0 0.0 0.0 0.0 0.0 0.0 1.0" | nc -u -w1 localhost 8888
```

**System Status Monitoring**:
The VR control client provides real-time debug output:
```
Cycle 1: Activation: 0.000, IK success: yes
Cycle 2: Activation: 0.002, IK success: yes
...
Target vel: 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 [activation: 1.000]
```

**Performance Indicators**:
- **IK Success Rate**: Should be >95% during normal operation
- **Activation Factor**: Ramps from 0.000 to 1.000 over 0.5s
- **Target Velocities**: Range from 0.0 to ±2.0 rad/s depending on motion

## Troubleshooting

### Common Issues:

**VR data not received**:
- Check VR app is sending to port **8888** (not 9000)
- Verify firewall allows UDP traffic: `sudo ufw allow 8888/udp`
- Test with netcat: `nc -u -l 8888` and check for VR messages
- Ensure message format is: `x y z qx qy qz qw` (7 space-separated numbers)

**Robot not responding**:
- Check robot is in ready state and not in protective stop
- Ensure external activation device is connected and activated
- Verify joint positions are within safe limits
- Check that `VR initialized!` message appeared

**Movement too sluggish**:
- Decrease `vr_smoothing` from 0.05 to 0.02 for less filtering
- Increase joint accelerations from 4.0-6.0 to 6.0-8.0 rad/s²
- Reduce `ACTIVATION_TIME_SEC` from 0.5 to 0.2 seconds
- Decrease position deadzone from 0.001 to 0.0005

**Control errors eliminated**:
- **No more reflex triggers**: Joint velocity control prevents discontinuities
- **Smooth startup**: Gradual activation prevents sudden motions  
- **Robust IK**: Weighted solver handles near-singular configurations
- **Continuous operation**: System designed for extended use without errors

**Jittery or unstable movement**:
- Increase `vr_smoothing` from 0.05 to 0.1 for more filtering
- Increase position deadzone from 0.001 to 0.005 to reduce noise sensitivity
- Check VR tracking quality and reduce environmental interference

### Network Protocol:
**VR → Robot Client**: Direct UDP messages (port 8888):
```
"x y z qx qy qz qw"
```
Example: `0.123 0.456 0.789 0.0 0.0 0.0 1.0`

## Quick Start Summary

1. **Build and run VR robot client**:
   ```bash
   cd vr_robot_client/build
   cmake .. && make -j4
   ./franka_vr_control_client <robot-hostname>
   ```

2. **Start VR application** sending pose data to port 8888

**This system provides professional-grade VR teleoperation with smooth, responsive control and robust error handling. The advanced motion control pipeline ensures safe, precise robot operation without the velocity discontinuities that plagued earlier systems.**

## Demo