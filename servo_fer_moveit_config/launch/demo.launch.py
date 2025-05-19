from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fer", package_name="servo_fer_moveit_config").to_moveit_configs()
    description = generate_demo_launch(moveit_config)

    # We need to start the gripper separately because it is not implemented as a ROS 2 controller, but rather is a separate node
    description.add_action(IncludeLaunchDescription(PathJoinSubstitution([FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py']),
                                    launch_arguments={
                                        'arm_id' : 'fer',
                                        'robot_ip': 'None',
                                        'use_fake_hardware' : 'true'}.items()))
    # We need a joint_state_publisher to unify the joint states from the gripper and the arm
    description.add_action(
        Node(package='joint_state_publisher',
             executable='joint_state_publisher',
             parameters=[{'source_list': ['joint_state_broadcaster/joint_states', 'fer_gripper/joint_states'], 'rate': 30}]))
    return description

