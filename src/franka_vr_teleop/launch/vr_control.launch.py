#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    vr_udp_ip_arg = DeclareLaunchArgument(
        'vr_udp_ip',
        default_value='0.0.0.0',
        description='IP address to listen for VR data'
    )
    
    vr_udp_port_arg = DeclareLaunchArgument(
        'vr_udp_port',
        default_value='9000',
        description='Port to listen for VR data'
    )
    
    robot_udp_ip_arg = DeclareLaunchArgument(
        'robot_udp_ip',
        default_value='192.168.18.1',
        description='IP address of the realtime PC running the Franka controller'
    )
    
    robot_udp_port_arg = DeclareLaunchArgument(
        'robot_udp_port',
        default_value='8888',
        description='Port number for communication with realtime PC'
    )
    
    pose_scale_arg = DeclareLaunchArgument(
        'pose_scale',
        default_value='0.5',
        description='Scale factor for VR hand movements'
    )
    
    orientation_scale_arg = DeclareLaunchArgument(
        'orientation_scale',
        default_value='1.0',
        description='Scale factor for VR hand rotations'
    )
    
    position_deadzone_arg = DeclareLaunchArgument(
        'position_deadzone',
        default_value='0.01',
        description='Deadzone for VR hand position (meters)'
    )
    
    orientation_deadzone_arg = DeclareLaunchArgument(
        'orientation_deadzone',
        default_value='0.05',
        description='Deadzone for VR hand orientation (radians)'
    )
    
    # VR to Robot converter node
    vr_converter_node = Node(
        package='franka_vr_teleop',
        executable='vr_to_robot_converter',
        name='vr_to_robot_converter',
        parameters=[{
            'vr_udp_ip': LaunchConfiguration('vr_udp_ip'),
            'vr_udp_port': LaunchConfiguration('vr_udp_port'),
            'robot_udp_ip': LaunchConfiguration('robot_udp_ip'),
            'robot_udp_port': LaunchConfiguration('robot_udp_port'),
            'pose_scale': LaunchConfiguration('pose_scale'),
            'orientation_scale': LaunchConfiguration('orientation_scale'),
            'position_deadzone': LaunchConfiguration('position_deadzone'),
            'orientation_deadzone': LaunchConfiguration('orientation_deadzone'),
            'smoothing_factor': 0.8,
            'control_rate': 50.0,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        vr_udp_ip_arg,
        vr_udp_port_arg,
        robot_udp_ip_arg,
        robot_udp_port_arg,
        pose_scale_arg,
        orientation_scale_arg,
        position_deadzone_arg,
        orientation_deadzone_arg,
        vr_converter_node,
    ])