#!/usr/bin/env python3
# vr_to_robot_converter.py - Convert VR wrist tracking to robot EE commands

import rclpy
from rclpy.node import Node
import socket
import threading
import time
import re
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped, Twist
import tf2_ros
from tf2_ros import TransformException

class VRToRobotConverter(Node):
    def __init__(self):
        super().__init__('vr_to_robot_converter')
        
        # Parameters
        self.declare_parameter('vr_udp_ip', '0.0.0.0')
        self.declare_parameter('vr_udp_port', 9999)
        self.declare_parameter('robot_udp_ip', '192.168.18.1')
        self.declare_parameter('robot_udp_port', 8888)
        self.declare_parameter('pose_scale', 2.0)  # Scale factor for VR movements
        self.declare_parameter('orientation_scale', 1.0)  # Scale factor for VR rotations
        self.declare_parameter('smoothing_factor', 0.8)  # Smoothing for VR data
        self.declare_parameter('control_rate', 50.0)  # Hz
        self.declare_parameter('position_deadzone', 0.01)  # 1cm deadzone for position
        self.declare_parameter('orientation_deadzone', 0.05)  # 0.05 rad (~3°) deadzone for orientation
        
        # Get parameters
        self.vr_udp_ip = self.get_parameter('vr_udp_ip').value
        self.vr_udp_port = self.get_parameter('vr_udp_port').value
        self.robot_udp_ip = self.get_parameter('robot_udp_ip').value
        self.robot_udp_port = self.get_parameter('robot_udp_port').value
        self.pose_scale = self.get_parameter('pose_scale').value
        self.orientation_scale = self.get_parameter('orientation_scale').value
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        self.control_rate = self.get_parameter('control_rate').value
        self.position_deadzone = self.get_parameter('position_deadzone').value
        self.orientation_deadzone = self.get_parameter('orientation_deadzone').value
        
        # VR wrist tracking state
        self.current_vr_pose = None
        self.initial_vr_pose = None
        self.vr_data_received = False
        
        # Robot state
        self.robot_initial_pose = None
        self.current_robot_target = None
        
        # Smoothing
        self.smoothed_position = np.array([0.0, 0.0, 0.0])
        self.smoothed_orientation = np.array([0.0, 0.0, 0.0])  # axis-angle representation
        
        # VR UDP pattern matching
        self.wrist_pattern = re.compile(r'Right wrist:, ([-\d\.]+), ([-\d\.]+), ([-\d\.]+), ([-\d\.]+), ([-\d\.]+), ([-\d\.]+), ([-\d\.]+)')
        
        # Setup VR UDP receiver
        self.vr_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.vr_socket.bind((self.vr_udp_ip, self.vr_udp_port))
        self.vr_socket.setblocking(False)
        
        # Setup robot UDP sender
        self.robot_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Start VR receiver thread
        self.vr_thread = threading.Thread(target=self.receive_vr_data)
        self.vr_thread.daemon = True
        self.vr_thread.start()
        
        # Control loop timer
        self.timer = self.create_timer(1.0/self.control_rate, self.control_loop)
        
        # Publishers for debugging/visualization
        self.vr_pose_pub = self.create_publisher(PoseStamped, 'vr_wrist_pose', 10)
        self.robot_target_pub = self.create_publisher(PoseStamped, 'robot_target_pose', 10)
        
        # Logging
        self.command_counter = 0
        self.log_interval = 50  # Log every 50 cycles (1 second at 50Hz)
        
        self.get_logger().info(f"VR to Robot Converter started")
        self.get_logger().info(f"VR UDP: {self.vr_udp_ip}:{self.vr_udp_port}")
        self.get_logger().info(f"Robot UDP: {self.robot_udp_ip}:{self.robot_udp_port}")
        self.get_logger().info("Move your VR hand to start control!")
    
    def receive_vr_data(self):
        """Thread function to receive VR wrist tracking data"""
        self.get_logger().info('VR UDP receiver thread started')
        
        while rclpy.ok():
            try:
                data, addr = self.vr_socket.recvfrom(1024)
                message = data.decode('utf-8')
                self.parse_vr_message(message)
            except BlockingIOError:
                time.sleep(0.001)
            except Exception as e:
                self.get_logger().error(f'Error receiving VR data: {str(e)}')
    
    def parse_vr_message(self, message):
        """Parse VR wrist tracking message"""
        try:
            match = self.wrist_pattern.search(message)
            if match:
                # Extract raw VR wrist data
                x = float(match.group(1))
                y = float(match.group(2))
                z = float(match.group(3))
                qx = float(match.group(4))
                qy = float(match.group(5))
                qz = float(match.group(6))
                qw = float(match.group(7))
                
                # Transform VR coordinates to robot coordinates
                # VR: +x=right, +y=up, +z=forward → Robot: +x=forward, +y=left, +z=up
                robot_position = np.array([z, -x, y])  # VR z→robot x, VR -x→robot y, VR y→robot z
                
                # Transform quaternion from VR to robot frame
                vr_quat = np.array([qx, qy, qz, qw])
                robot_quat = self.transform_quaternion_vr_to_robot(vr_quat)
                
                # Store current VR pose
                self.current_vr_pose = {
                    'position': robot_position,
                    'orientation': robot_quat
                }
                
                # Set initial pose on first data
                if not self.vr_data_received:
                    self.initial_vr_pose = self.current_vr_pose.copy()
                    self.vr_data_received = True
                    self.get_logger().info("Initial VR pose captured!")
                
                # Publish VR pose for visualization
                self.publish_vr_pose(robot_position, robot_quat)
                
        except Exception as e:
            self.get_logger().error(f'Error parsing VR message: {str(e)}')
    
    def apply_deadzone(self, values, deadzone):
        """Apply deadzone to a vector - zero out values below threshold"""
        if isinstance(values, np.ndarray):
            result = values.copy()
            # Apply deadzone per component
            for i in range(len(result)):
                if abs(result[i]) < deadzone:
                    result[i] = 0.0
            return result
        else:
            # Single value
            return 0.0 if abs(values) < deadzone else values
    
    def transform_quaternion_vr_to_robot(self, vr_quat):
        """Transform quaternion from VR coordinate system to robot coordinate system"""
        # Create rotation from VR quaternion
        vr_rot = Rotation.from_quat(vr_quat)
        
        # Define coordinate system transformation: VR → Robot
        # VR: +x=right, +y=up, +z=forward
        # Robot: +x=forward, +y=left, +z=up
        coord_transform = Rotation.from_euler('xyz', [np.pi/2, 0, -np.pi/2])
        
        # Apply transformation
        robot_rot = coord_transform * vr_rot
        return robot_rot.as_quat()
    
    def control_loop(self):
        """Main control loop - converts VR pose to robot commands"""
        if not self.vr_data_received or self.current_vr_pose is None:
            return
        
        self.command_counter += 1
        
        try:
            # Calculate pose difference from initial VR pose
            vr_pos_delta = self.current_vr_pose['position'] - self.initial_vr_pose['position']
            
            # Apply deadzone to position delta
            vr_pos_delta = self.apply_deadzone(vr_pos_delta, self.position_deadzone)
            
            # Calculate orientation difference
            initial_rot = Rotation.from_quat(self.initial_vr_pose['orientation'])
            current_rot = Rotation.from_quat(self.current_vr_pose['orientation'])
            relative_rot = current_rot * initial_rot.inv()
            orientation_delta = relative_rot.as_rotvec()
            
            # Apply deadzone to orientation delta
            orientation_delta = self.apply_deadzone(orientation_delta, self.orientation_deadzone)
            
            # Scale the movements
            scaled_pos_delta = vr_pos_delta * self.pose_scale
            scaled_orient_delta = orientation_delta * self.orientation_scale
            
            # Apply smoothing
            self.smoothed_position = (self.smoothing_factor * self.smoothed_position + 
                                    (1 - self.smoothing_factor) * scaled_pos_delta)
            
            # For orientation, smooth in axis-angle space
            smoothed_orient_delta = (self.smoothing_factor * self.smoothed_orientation + 
                                   (1 - self.smoothing_factor) * scaled_orient_delta)
            
            # Update the stored smoothed orientation
            self.smoothed_orientation = smoothed_orient_delta
            
            # Send robot command
            self.send_robot_command(self.smoothed_position, smoothed_orient_delta)
            
            # Publish target pose for visualization
            self.publish_robot_target(self.smoothed_position, smoothed_orient_delta)
            
            # Logging
            if self.command_counter % self.log_interval == 0:
                self.get_logger().info(
                    f'VR→Robot: pos_delta=[{self.smoothed_position[0]:.3f}, {self.smoothed_position[1]:.3f}, {self.smoothed_position[2]:.3f}], '
                    f'orient_delta=[{smoothed_orient_delta[0]:.3f}, {smoothed_orient_delta[1]:.3f}, {smoothed_orient_delta[2]:.3f}]'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')
    
    def send_robot_command(self, position_delta, orientation_delta):
        """Send pose command to robot via UDP"""
        try:
            # Convert to velocity-like commands (delta per time step)
            dt = 1.0 / self.control_rate
            linear_vel = position_delta / dt
            angular_vel = orientation_delta / dt
            
            # Create UDP message in format expected by robot
            # "linear_x linear_y linear_z angular_x angular_y angular_z emergency_stop reset_pose"
            message = f"{linear_vel[0]:.6f} {linear_vel[1]:.6f} {linear_vel[2]:.6f} " + \
                     f"{angular_vel[0]:.6f} {angular_vel[1]:.6f} {angular_vel[2]:.6f} 0 0"
            
            # Send to robot
            self.robot_socket.sendto(message.encode(), (self.robot_udp_ip, self.robot_udp_port))
            
        except Exception as e:
            self.get_logger().error(f'Error sending robot command: {str(e)}')
    
    def publish_vr_pose(self, position, orientation):
        """Publish VR pose for visualization"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "vr_tracking"
        
        # Ensure all values are Python floats, not numpy types
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])
        
        pose_msg.pose.orientation.x = float(orientation[0])
        pose_msg.pose.orientation.y = float(orientation[1])
        pose_msg.pose.orientation.z = float(orientation[2])
        pose_msg.pose.orientation.w = float(orientation[3])
        
        self.vr_pose_pub.publish(pose_msg)
    
    def publish_robot_target(self, position_delta, orientation_delta):
        """Publish robot target pose for visualization"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "robot_target"
        
        # Ensure all values are Python floats
        pose_msg.pose.position.x = float(position_delta[0])
        pose_msg.pose.position.y = float(position_delta[1])
        pose_msg.pose.position.z = float(position_delta[2])
        
        # Convert axis-angle to quaternion for visualization
        if np.linalg.norm(orientation_delta) > 1e-6:
            target_rot = Rotation.from_rotvec(orientation_delta)
            target_quat = target_rot.as_quat()
        else:
            target_quat = np.array([0, 0, 0, 1])
        
        pose_msg.pose.orientation.x = float(target_quat[0])
        pose_msg.pose.orientation.y = float(target_quat[1])
        pose_msg.pose.orientation.z = float(target_quat[2])
        pose_msg.pose.orientation.w = float(target_quat[3])
        
        self.robot_target_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = VRToRobotConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()