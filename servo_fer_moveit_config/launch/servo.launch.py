from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Package directories
    franka_description_pkg = get_package_share_directory('franka_description')
    franka_moveit_config_pkg = get_package_share_directory('servo_fer_moveit_config')
    
    # Get paths to relevant files
    franka_xacro_file = os.path.join(
        franka_description_pkg,
        'robots',
        'fer/fer.urdf.xacro'
    )
    
    # Use ParameterValue to handle command substitution
    robot_description = ParameterValue(
        Command([
            'xacro', ' ', franka_xacro_file, ' ',
            'arm_id:=fer',
        ]),
        value_type=str
    )
    
    # Get semantic description (SRDF)
    robot_description_semantic_file = os.path.join(
        franka_moveit_config_pkg,
        'srdf',
        'fer_arm.srdf.xacro'
    )
    
    # Also process SRDF with xacro
    robot_description_semantic = ParameterValue(
        Command([
            'xacro', ' ', robot_description_semantic_file, ' ',
            'arm_id:=fer',
        ]),
        value_type=str
    )
    
    # Get kinematics config
    kinematics_yaml = os.path.join(
        franka_moveit_config_pkg,
        'config',
        'kinematics.yaml'
    )
    
    # Load kinematics yaml
    with open(kinematics_yaml, 'r') as f:
        kinematics_params = yaml.safe_load(f)
    
    # Get servo config
    servo_params_file = os.path.join(
        franka_moveit_config_pkg,
        'config',
        'servo_config.yaml'
    )
    
    # Load servo params yaml and modify
    with open(servo_params_file, 'r') as f:
        servo_params = yaml.safe_load(f)
    
    # Ensure all necessary parameters are set
    if 'moveit_servo' not in servo_params:
        servo_params['moveit_servo'] = {}
    
    # Set the command type explicitly for pose-based control
    servo_params['moveit_servo']['command_in_type'] = 'speed_units'
    
    # Set the end-effector frame explicitly
    servo_params['moveit_servo']['ee_frame_name'] = 'fer_link8'
    
    # Set the planning frame
    servo_params['moveit_servo']['planning_frame'] = 'fer_link0'
    
    # Explicitly set parameters for pose control
    servo_params['moveit_servo']['pose_command_in_topic'] = '~/pose_target'
    
    return LaunchDescription([
        # Servo node with updated config
        Node(
            package='moveit_servo',
            executable='servo_node',
            name='servo_node',
            output='screen',
            parameters=[
                servo_params,
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic},
                {'robot_description_kinematics': kinematics_params},
                # Additional explicit parameters for pose-based control
                {'moveit_servo.command_in_type': 'speed_units'},
                {'moveit_servo.use_pose_command_interface': True},  # Enable pose command interface
            ],
        ),
    ])