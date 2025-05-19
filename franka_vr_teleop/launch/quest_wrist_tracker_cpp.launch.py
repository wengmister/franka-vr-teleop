from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Use parameters from the Python launch file, or define new defaults
    # For consistency, let's use the last good known Python launch params
    # Ensure these match your desired C++ node parameters
    
    # Default parameters (can be overridden by command-line arguments to ros2 launch)
    default_udp_port = '9000'
    default_udp_ip = '0.0.0.0'
    default_planning_frame = 'fer_link0'
    default_ee_frame = 'fer_link8'
    default_pose_change_scale = '0.4'
    default_max_position_change = '0.06'
    default_max_orientation_change = '0.10'
    default_position_smoothing = '0.75'
    default_orientation_smoothing = '0.75'

    return LaunchDescription([
        DeclareLaunchArgument('udp_port', default_value=default_udp_port),
        DeclareLaunchArgument('udp_ip', default_value=default_udp_ip),
        DeclareLaunchArgument('planning_frame', default_value=default_planning_frame),
        DeclareLaunchArgument('ee_frame', default_value=default_ee_frame),
        DeclareLaunchArgument('pose_change_scale', default_value=default_pose_change_scale),
        DeclareLaunchArgument('max_position_change', default_value=default_max_position_change),
        DeclareLaunchArgument('max_orientation_change', default_value=default_max_orientation_change),
        DeclareLaunchArgument('position_smoothing', default_value=default_position_smoothing),
        DeclareLaunchArgument('orientation_smoothing', default_value=default_orientation_smoothing),

        Node(
            package='franka_vr_teleop',
            executable='wrist_pose_control_cpp_node', # Matches CMakeLists.txt
            name='wrist_servo_pose_control_cpp',    # Matches C++ node name
            output='screen',
            parameters=[{
                'udp_port': LaunchConfiguration('udp_port'),
                'udp_ip': LaunchConfiguration('udp_ip'),
                'planning_frame': LaunchConfiguration('planning_frame'),
                'ee_frame': LaunchConfiguration('ee_frame'),
                'pose_change_scale': LaunchConfiguration('pose_change_scale'),
                'max_position_change': LaunchConfiguration('max_position_change'),
                'max_orientation_change': LaunchConfiguration('max_orientation_change'),
                'position_smoothing': LaunchConfiguration('position_smoothing'),
                'orientation_smoothing': LaunchConfiguration('orientation_smoothing'),
            }],
        ),
    ]) 