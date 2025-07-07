#!/usr/bin/env python3
# vr_to_robot_converter.py - Convert VR wrist tracking to robot pose commands

import rclpy
from rclpy.node import Node
import socket
import threading
import time
import re
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from geometry_msgs.msg import PoseStamped

class VRToRobotConverter(Node):
    def __init__(self):
        super().__init__('vr_to_robot_converter')
        
        # Parameters
        self.declare_parameter('vr_udp_ip', '0.0.0.0')
        self.declare_parameter('vr_udp_port', 9999)
        self.declare_parameter('robot_udp_ip', '192.168.18.1')
        self.declare_parameter('robot_udp_port', 8888)
        self.declare_parameter('pose_scale', 1.0)
        self.declare_parameter('orientation_scale', 1.0)
        self.declare_parameter('smoothing_factor', 0.4)
        self.declare_parameter('control_rate', 50.0)  # Hz
        
        # Get parameters
        self.vr_udp_ip = self.get_parameter('vr_udp_ip').value
        self.vr_udp_port = self.get_parameter('vr_udp_port').value
        self.robot_udp_ip = self.get_parameter('robot_udp_ip').value
        self.robot_udp_port = self.get_parameter('robot_udp_port').value
        self.pose_scale = self.get_parameter('pose_scale').value
        self.orientation_scale = self.get_parameter('orientation_scale').value
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        self.control_rate = self.get_parameter('control_rate').value
        
        # VR state
        self.current_vr_pose = None
        self.initial_vr_pose = None
        self.vr_data_received = False
        
        # Robot state - absolute pose (not delta)
        self.robot_base_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0])  # identity quaternion
        }
        
        # Smoothing
        self.smoothed_position = np.array([0.0, 0.0, 0.0])
        self.smoothed_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        
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
        
        # Publishers for visualization
        self.vr_pose_pub = self.create_publisher(PoseStamped, 'vr_wrist_pose', 10)
        self.robot_target_pub = self.create_publisher(PoseStamped, 'robot_target_pose', 10)
        
        # Frequency tracking
        self.command_counter = 0
        self.last_log_time = time.time()
        self.commands_sent = 0
        self.vr_messages_received = 0  # Track VR message frequency
        self.log_interval = 2.0  # Log every 2 seconds
        
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
                # Increment VR message counter
                self.vr_messages_received += 1

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
                robot_position = np.array([z, -x, y])
                
                # Transform quaternion from VR to robot frame
                # VR is left-handed, robot is right-handed - need reflection + rotation
                vr_rot = Rotation.from_quat([qx, qy, qz, qw])
                
                # Convert VR rotation to rotation matrix
                vr_matrix = vr_rot.as_matrix()
                
                # Define coordinate transformation matrix from VR to robot
                # VR: [right, up, forward] → Robot: [forward, left, up]  
                # This maps: VR_x→Robot_y, VR_y→Robot_z, VR_z→Robot_x
                # Include handedness flip by negating one axis
                transform_matrix = np.array([
                    [0,  0,  1],  # Robot X = VR Z (forward)
                    [-1, 0,  0],  # Robot Y = -VR X (left = -right)  
                    [0,  1,  0]   # Robot Z = VR Y (up)
                ])
                
                # Apply transformation: R_robot = T * R_vr * T^-1
                robot_matrix = transform_matrix @ vr_matrix @ transform_matrix.T
                
                # Convert back to quaternion
                robot_rot = Rotation.from_matrix(robot_matrix)
                robot_quat = robot_rot.as_quat()
                
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
    

    def control_loop(self):
        """Main control loop - converts VR pose to robot commands"""
        if not self.vr_data_received or self.current_vr_pose is None:
            return
        
        self.command_counter += 1
        
        try:
            # Calculate pose difference from initial VR pose
            vr_pos_delta = self.current_vr_pose['position'] - self.initial_vr_pose['position']
            
            # Scale the movements
            scaled_pos_delta = vr_pos_delta * self.pose_scale
            
            # Apply smoothing
            self.smoothed_position = (self.smoothing_factor * self.smoothed_position + 
                                    (1 - self.smoothing_factor) * scaled_pos_delta)
            
            # Calculate orientation difference as axis-angle for scaling
            initial_rot = Rotation.from_quat(self.initial_vr_pose['orientation'])
            current_rot = Rotation.from_quat(self.current_vr_pose['orientation'])
            relative_rot = current_rot * initial_rot.inv()
            
            # Scale the relative rotation
            relative_rotvec = relative_rot.as_rotvec() * self.orientation_scale
            scaled_relative_rot = Rotation.from_rotvec(relative_rotvec)
            
            # Apply scaled relative rotation to initial pose
            scaled_orient_rot = initial_rot * scaled_relative_rot
            
            # For orientation, interpolate quaternions using Slerp
            target_rot = scaled_orient_rot
            
            # Slerp between current and target orientation
            slerp_t = 1 - self.smoothing_factor
            key_rotations = Rotation.from_quat([self.smoothed_orientation, target_rot.as_quat()])
            slerp = Slerp([0, 1], key_rotations)
            smoothed_rot = slerp(slerp_t)
            self.smoothed_orientation = smoothed_rot.as_quat()
            
            # Calculate absolute target pose (base + delta)
            target_position = self.robot_base_pose['position'] + self.smoothed_position
            target_orientation = self.smoothed_orientation
            
            # Send robot command
            self.send_robot_command(target_position, target_orientation)
            
            # Publish target pose for visualization
            self.publish_robot_target(target_position, target_orientation)
            
            # Frequency logging
            current_time = time.time()
            if current_time - self.last_log_time >= self.log_interval:
                command_frequency = self.commands_sent / (current_time - self.last_log_time)
                vr_frequency = self.vr_messages_received / (current_time - self.last_log_time)
                self.get_logger().info(
                    f'VR input: {vr_frequency:.1f} Hz | Command output: {command_frequency:.1f} Hz | '
                    f'Target pos: [{target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f}] | '
                    f'VR delta: [{self.smoothed_position[0]:.3f}, {self.smoothed_position[1]:.3f}, {self.smoothed_position[2]:.3f}]'
                )
                self.last_log_time = current_time
                self.commands_sent = 0
                self.vr_messages_received = 0
            
            self.commands_sent += 1
                
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')
    
    def send_robot_command(self, position, orientation):
        """Send absolute pose command to robot via UDP"""
        try:
            # Send absolute pose: position (x,y,z) and orientation (qx,qy,qz,qw)
            message = f"{position[0]:.6f} {position[1]:.6f} {position[2]:.6f} " + \
                     f"{orientation[0]:.6f} {orientation[1]:.6f} {orientation[2]:.6f} {orientation[3]:.6f}"
            
            # Send to robot
            self.robot_socket.sendto(message.encode(), (self.robot_udp_ip, self.robot_udp_port))
            
        except Exception as e:
            self.get_logger().error(f'Error sending robot command: {str(e)}')
    
    def publish_vr_pose(self, position, orientation):
        """Publish VR pose for visualization"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"  # Use map as parent frame
        
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])
        
        pose_msg.pose.orientation.x = float(orientation[0])
        pose_msg.pose.orientation.y = float(orientation[1])
        pose_msg.pose.orientation.z = float(orientation[2])
        pose_msg.pose.orientation.w = float(orientation[3])
        
        self.vr_pose_pub.publish(pose_msg)
    
    def publish_robot_target(self, position, orientation):
        """Publish robot target pose for visualization"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"  # Use map as parent frame
        
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])
        
        pose_msg.pose.orientation.x = float(orientation[0])
        pose_msg.pose.orientation.y = float(orientation[1])
        pose_msg.pose.orientation.z = float(orientation[2])
        pose_msg.pose.orientation.w = float(orientation[3])
        
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