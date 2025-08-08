#!/usr/bin/env python3
# vr_to_robot_converter.py - Convert VR wrist tracking to robot pose commands

import rclpy
from rclpy.node import Node
import socket
import threading
import time
import sys
import select
import re
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from geometry_msgs.msg import PoseStamped

class VRToRobotConverter(Node):
    def __init__(self):
        super().__init__('vr_to_robot_converter')
        
        # Parameters
        self.declare_parameter('use_tcp', False)
        self.declare_parameter('vr_tcp_port', 8000)
        self.declare_parameter('vr_udp_ip', '0.0.0.0')
        self.declare_parameter('vr_udp_port', 9999)
        self.declare_parameter('robot_udp_ip', '192.168.18.1')
        self.declare_parameter('robot_udp_port', 8888)
        self.declare_parameter('smoothing_factor', 0.7)
        self.declare_parameter('control_rate', 100.0)  # Hz
        self.declare_parameter('pause_enabled', False)
        
        # Get parameters
        self.use_tcp = self.get_parameter('use_tcp').value
        self.vr_tcp_port = self.get_parameter('vr_tcp_port').value
        self.vr_udp_ip = self.get_parameter('vr_udp_ip').value
        self.vr_udp_port = self.get_parameter('vr_udp_port').value
        self.robot_udp_ip = self.get_parameter('robot_udp_ip').value
        self.robot_udp_port = self.get_parameter('robot_udp_port').value
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        self.control_rate = self.get_parameter('control_rate').value
        self.pause_enabled = self.get_parameter('pause_enabled').value
        
        # VR state
        self.current_vr_pose = None
        self.initial_vr_pose = None
        self.vr_data_received = False

        # Target robot pose
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.target_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Robot state - absolute pose (not delta)
        self.robot_base_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0])  # identity quaternion
        }
        
        # Smoothing
        self.smoothed_position = np.array([0.0, 0.0, 0.0])
        self.smoothed_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Fist state control
        self.fist_state = 'open'  # 'open', 'closed', or 'unknown'
        self.is_paused = False
        self.paused_vr_pose = {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([0.0, 0.0, 0.0, 1.0])
                }
        
        # VR UDP pattern matching
        self.wrist_pattern = re.compile(r'Right wrist:, ([-\d\.]+), ([-\d\.]+), ([-\d\.]+), ([-\d\.]+), ([-\d\.]+), ([-\d\.]+), ([-\d\.]+), leftFist: (\w+)')
        
        # TCP-specific state
        self.tcp_socket = None
        self.tcp_connection = None
        self.tcp_client_address = None
        
        # Setup VR receiver (UDP or TCP)
        if self.use_tcp:
            # Setup adb reverse port forwarding first
            if not self.setup_adb_reverse(self.vr_tcp_port):
                self.get_logger().warn("adb reverse setup failed, but continuing with TCP socket setup")
                self.get_logger().warn("You may need to run 'adb reverse tcp:{port} tcp:{port}' manually".format(port=self.vr_tcp_port))
            
            try:
                self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.tcp_socket.bind(('localhost', self.vr_tcp_port))
                self.tcp_socket.listen(1)  # Allow one connection
                self.get_logger().info(f"Successfully bound VR TCP socket to localhost:{self.vr_tcp_port}")
            except Exception as e:
                self.get_logger().error(f"Failed to setup VR TCP socket: {str(e)}")
                # Clean up adb reverse if socket setup failed
                self.cleanup_adb_reverse(self.vr_tcp_port)
                raise
        else:
            try:
                self.vr_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.vr_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.vr_socket.bind((self.vr_udp_ip, self.vr_udp_port))
                self.vr_socket.setblocking(False)
                self.get_logger().info(f"Successfully bound VR UDP socket to {self.vr_udp_ip}:{self.vr_udp_port}")
            except Exception as e:
                self.get_logger().error(f"Failed to setup VR UDP socket: {str(e)}")
                raise
        
        # Setup robot UDP sender
        self.robot_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Start VR receiver thread
        if self.use_tcp:
            self.vr_thread = threading.Thread(target=self.receive_vr_data_tcp)
        else:
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
        if self.use_tcp:
            self.get_logger().info(f"VR TCP: localhost:{self.vr_tcp_port}")
        else:
            self.get_logger().info(f"VR UDP: {self.vr_udp_ip}:{self.vr_udp_port}")
        self.get_logger().info(f"Robot UDP: {self.robot_udp_ip}:{self.robot_udp_port}")
        self.get_logger().info("Move your VR hand to start control!")

    def __del__(self):
        """Cleanup resources when node is destroyed"""
        if hasattr(self, 'use_tcp') and self.use_tcp and hasattr(self, 'vr_tcp_port'):
            self.cleanup_adb_reverse(self.vr_tcp_port)
    
    def receive_vr_data(self):
        """Thread function to receive VR wrist tracking data"""
        self.get_logger().info('VR UDP receiver thread started')
        last_debug_time = time.time()
        message_count = 0
        
        while rclpy.ok():
            try:
                data, addr = self.vr_socket.recvfrom(1024)
                message = data.decode('utf-8')
                message_count += 1
                
                # Debug logging every 5 seconds
                current_time = time.time()
                if current_time - last_debug_time >= 5.0:
                    self.get_logger().info(f"Received {message_count} VR messages in last 5 seconds")
                    self.get_logger().info(f"Latest message from {addr}: {message[:100]}...")  # First 100 chars
                    last_debug_time = current_time
                    message_count = 0
                
                self.parse_vr_message(message)
            except BlockingIOError:
                # No data available, check for debug timeout
                current_time = time.time()
                if current_time - last_debug_time >= 5.0:
                    if message_count == 0:  # Only log if truly no messages received
                        self.get_logger().info("No VR data received in last 5 seconds")
                    last_debug_time = current_time
                    message_count = 0
                time.sleep(0.001)
            except Exception as e:
                self.get_logger().error(f'Error receiving VR data: {str(e)}')

    def receive_vr_data_tcp(self):
        """Thread function to receive VR wrist tracking data via TCP"""
        self.get_logger().info('VR TCP receiver thread started')
        last_debug_time = time.time()
        message_count = 0
        
        while rclpy.ok():
            try:
                # Wait for a connection if we don't have one
                if self.tcp_connection is None:
                    self.get_logger().info("Waiting for TCP connection...")
                    self.tcp_connection, self.tcp_client_address = self.tcp_socket.accept()
                    self.tcp_connection.settimeout(1.0)  # 1 second timeout for recv
                    self.get_logger().info(f"TCP connection established from {self.tcp_client_address}")

                # Try to receive data
                try:
                    data = self.tcp_connection.recv(1024)
                    if not data:
                        # Connection closed by client
                        self.get_logger().info("TCP client disconnected")
                        self.tcp_connection.close()
                        self.tcp_connection = None
                        self.tcp_client_address = None
                        continue
                    
                    message = data.decode('utf-8')
                    message_count += 1
                    
                    # Debug logging every 5 seconds
                    current_time = time.time()
                    if current_time - last_debug_time >= 5.0:
                        self.get_logger().info(f"Received {message_count} VR messages in last 5 seconds")
                        self.get_logger().info(f"Latest message from {self.tcp_client_address}: {message[:100]}...")  # First 100 chars
                        last_debug_time = current_time
                        message_count = 0
                    
                    self.parse_vr_message(message)
                    
                except socket.timeout:
                    # No data received within timeout, check for debug timeout
                    current_time = time.time()
                    if current_time - last_debug_time >= 5.0:
                        if message_count == 0:  # Only log if truly no messages received
                            self.get_logger().info("No VR data received in last 5 seconds")
                        last_debug_time = current_time
                        message_count = 0
                    continue
                    
                except ConnectionResetError:
                    self.get_logger().info("TCP connection reset by client")
                    self.tcp_connection.close()
                    self.tcp_connection = None
                    self.tcp_client_address = None
                    continue
                    
            except Exception as e:
                self.get_logger().error(f'Error in TCP receiver: {str(e)}')
                if self.tcp_connection:
                    self.tcp_connection.close()
                    self.tcp_connection = None
                    self.tcp_client_address = None
                time.sleep(1.0)  # Wait before retrying
    
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
                fist_state = match.group(8)
                
                # Update fist state
                self.fist_state = fist_state
                
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
            else:
                # Log messages that don't match our pattern (first few only)
                # Skip warning for "Right landmarks" messages as they are expected but not used
                if not message.startswith("Right landmarks"):
                    if not hasattr(self, '_pattern_miss_count'):
                        self._pattern_miss_count = 0
                    if self._pattern_miss_count < 5:
                        self.get_logger().warn(f"VR message doesn't match pattern: {message[:200]}")
                        self._pattern_miss_count += 1
                
        except Exception as e:
            self.get_logger().error(f'Error parsing VR message: {str(e)}')
            self.get_logger().error(f'Problem message: {message[:200]}')
    

    def control_loop(self):
        """Main control loop - converts VR pose to robot commands"""
        if not self.vr_data_received or self.current_vr_pose is None:
            return
        
        self.command_counter += 1
        
        try:
            # Handle fist state changes for pause/resume (only if pause is enabled)
            if self.pause_enabled:
                should_pause = self.fist_state == 'closed'
                
                if should_pause and not self.is_paused:
                    # Transition to paused state
                    self.is_paused = True
                    self.paused_vr_pose['position'] = self.target_position.copy()
                    self.paused_vr_pose['orientation'] = self.target_orientation.copy()

                    self.get_logger().info("Paused differential updates (fist closed)")
                    
                elif not should_pause and self.is_paused:
                    # Transition from paused to active state
                    self.is_paused = False

                    # Upon releasing, record the new initial pose
                    self.initial_vr_pose = self.current_vr_pose.copy()
                    self.get_logger().info("Resumed differential updates (fist open)")
                    
                # If paused, continue using the paused position/orientation
                if self.is_paused:
                    self.target_position = self.paused_vr_pose['position']
                    self.target_orientation = self.paused_vr_pose['orientation']
                else:
                    # Calculate pose difference from reference VR pose
                    vr_pos_delta = self.current_vr_pose['position'] - self.initial_vr_pose['position'] +  self.paused_vr_pose['position']
                    
                    # Apply smoothing to position
                    self.smoothed_position = (self.smoothing_factor * self.smoothed_position + 
                                            (1 - self.smoothing_factor) * vr_pos_delta)
                    
                    # Calculate orientation difference as relative rotation
                    initial_rot = Rotation.from_quat(self.initial_vr_pose['orientation'])
                    current_rot = Rotation.from_quat(self.current_vr_pose['orientation'])
                    # Calculate relative rotation from initial to current
                    relative_rot = current_rot * initial_rot.inv()

                    # Apply relative rotation from paused VR pose
                    paused_rot = Rotation.from_quat(self.paused_vr_pose['orientation'])
                    target_rot = paused_rot * relative_rot
                    
                    # Slerp between current smoothed orientation and target orientation
                    slerp_t = 1 - self.smoothing_factor
                    current_smoothed_rot = Rotation.from_quat(self.smoothed_orientation)
                    key_rotations = Rotation.from_quat([current_smoothed_rot.as_quat(), target_rot.as_quat()])
                    slerp = Slerp([0, 1], key_rotations)
                    smoothed_rot = slerp(slerp_t)
                    self.smoothed_orientation = smoothed_rot.as_quat()
                    
                    # Normalize quaternion to ensure it remains a unit quaternion
                    self.smoothed_orientation = self.smoothed_orientation / np.linalg.norm(self.smoothed_orientation)
                    
                    # Calculate absolute target pose (base + delta)
                    self.target_position = self.robot_base_pose['position'] + self.smoothed_position
                    
                    # Since base orientation is identity quaternion, we can directly use the smoothed relative orientation
                    self.target_orientation = self.smoothed_orientation
            else:
                # Pause disabled - always do normal differential calculation
                # Calculate pose difference from reference VR pose
                vr_pos_delta = self.current_vr_pose['position'] - self.initial_vr_pose['position']
                
                # Apply smoothing to position
                self.smoothed_position = (self.smoothing_factor * self.smoothed_position + 
                                        (1 - self.smoothing_factor) * vr_pos_delta)
                
                # Calculate orientation difference as relative rotation
                initial_rot = Rotation.from_quat(self.initial_vr_pose['orientation'])
                current_rot = Rotation.from_quat(self.current_vr_pose['orientation'])
                # Calculate relative rotation from initial to current
                relative_rot = current_rot * initial_rot.inv()
                
                # Slerp between current smoothed orientation and target orientation
                slerp_t = 1 - self.smoothing_factor
                current_smoothed_rot = Rotation.from_quat(self.smoothed_orientation)
                key_rotations = Rotation.from_quat([current_smoothed_rot.as_quat(), relative_rot.as_quat()])
                slerp = Slerp([0, 1], key_rotations)
                smoothed_rot = slerp(slerp_t)
                self.smoothed_orientation = smoothed_rot.as_quat()
                
                # Normalize quaternion to ensure it remains a unit quaternion
                self.smoothed_orientation = self.smoothed_orientation / np.linalg.norm(self.smoothed_orientation)
                
                # Calculate absolute target pose (base + delta)
                self.target_position = self.robot_base_pose['position'] + self.smoothed_position
                
                # Since base orientation is identity quaternion, we can directly use the smoothed relative orientation
                self.target_orientation = self.smoothed_orientation
            
            # Send robot command
            self.send_robot_command(self.target_position, self.target_orientation)
            
            # Publish target pose for visualization
            self.publish_robot_target(self.target_position, self.target_orientation)
            
            # Frequency logging
            current_time = time.time()
            if current_time - self.last_log_time >= self.log_interval:
                vr_frequency = self.vr_messages_received / (current_time - self.last_log_time)
                if self.pause_enabled:
                    pause_status = "PAUSED" if self.is_paused else "ACTIVE"
                    fist_info = f'Fist: {self.fist_state} ({pause_status})'
                else:
                    fist_info = 'Pause: DISABLED'
                self.get_logger().info(
                    f'VR UDP sampling: {vr_frequency:.1f} Hz | {fist_info} | '
                    f'Target pos: [{self.target_position[0]:.3f}, {self.target_position[1]:.3f}, {self.target_position[2]:.3f}] | '
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

    def setup_adb_reverse(self, port):
        """Setup adb reverse port forwarding"""
        try:
            # First check if adb is available
            result = subprocess.run(['adb', 'devices'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode != 0:
                self.get_logger().error("adb command not found. Please install Android SDK platform-tools.")
                return False
            
            # Check if device is connected
            if "device" not in result.stdout:
                self.get_logger().error("No Android device connected via adb.")
                return False
            
            # Setup reverse port forwarding
            cmd = ['adb', 'reverse', f'tcp:{port}', f'tcp:{port}']
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                self.get_logger().info(f"Successfully setup adb reverse tcp:{port} tcp:{port}")
                return True
            else:
                self.get_logger().error(f"Failed to setup adb reverse: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error("adb command timed out")
            return False
        except FileNotFoundError:
            self.get_logger().error("adb command not found. Please install Android SDK platform-tools.")
            return False
        except Exception as e:
            self.get_logger().error(f"Error setting up adb reverse: {str(e)}")
            return False

    def cleanup_adb_reverse(self, port):
        """Remove adb reverse port forwarding"""
        try:
            cmd = ['adb', 'reverse', '--remove', f'tcp:{port}']
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                self.get_logger().info(f"Successfully removed adb reverse tcp:{port}")
            else:
                self.get_logger().warn(f"Failed to remove adb reverse: {result.stderr}")
                
        except Exception as e:
            self.get_logger().warn(f"Error cleaning up adb reverse: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    
    node = VRToRobotConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up adb reverse if using TCP
        if node.use_tcp:
            node.cleanup_adb_reverse(node.vr_tcp_port)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()