# Franka VR Teleop

`ROS 2` package for teleoperating franka `FER` robot using Meta Quest 3s VR Headset. 

## VR Interface

`Unity` based VR interface can be found in [this repository](https://github.com/wengmister/quest-wrist-tracker)

## Franka Teleopration

Current implementation uses wrist vector captured from OXR Hand rig, transforms to robot base frame `fer_link0` and implements pose tracking via `moveit-servo` package. 

I noticed that there are quite a bit of residual error in orientation tracking - presumably due to the joint residual error generated from `moveit-servo` controller. I will reimplement this using `libfranka` in the near future.

Built on `ROS2 JAZZY`.

## Demo

Build, source install and launch:

    ros2 launch quest_wrist_tracker_cpp.launch.py

You will need to source your `franka_description` pkg. Official repo can be [found here](https://github.com/frankaemika/franka_description).

![frankavrtel-ezgif com-cut](https://github.com/user-attachments/assets/aa5c57f6-fdaa-4386-90b8-c516f7a92203)

## License

MIT
