#!/usr/bin/env python3
"""
VR-Based Cartesian Teleoperation with Automatic Error Recovery
Refactored from C++ libfranka to Python franky

Key improvements over C++ version:
- Simplified asynchronous motion preemption with intelligent handling of lower-frequency VR input
- Built-in trajectory optimization with Ruckig for smooth motion
- Python's superior error handling and networking
- Cleaner separation of concerns
"""

import asyncio
import socket
import threading
import time
import signal
import sys
import math
from dataclasses import dataclass, field
from typing import Optional, Tuple
import numpy as np
from scipy.spatial.transform import Rotation

try:
    from franky import (
        Robot, Affine, CartesianMotion, ReferenceType, 
        Measure, Reaction, RobotPose, ElbowState, RelativeDynamicsFactor,
        Gripper
    )
    # Franky 0.13.3's Affine properties: .translation (list), .matrix (16-element list, column-major)
except ImportError:
    print("Error: franky not installed. Run: 'pip install franky-panda'")
    sys.exit(1)


@dataclass
class VRCommand:
    """VR pose command structure"""
    pos_x: float = 0.0
    pos_y: float = 0.0  
    pos_z: float = 0.0
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0
    quat_w: float = 1.0
    has_valid_data: bool = False
    timestamp: float = 0.0 # Add a timestamp to detect new data


@dataclass 
class VRControlParams:
    """VR control parameters"""
    # Motion dynamics - these apply to the Ruckig trajectory generator
    # Tuned for more responsive and smooth motion
    velocity_factor: float = 0.5       # Increased for more responsiveness (range 0-1)
    acceleration_factor: float = 0.4   # Increased for faster response (range 0-1)
    jerk_factor: float = 0.015           # Increased for smoother acceleration changes (higher means more snappy, range 0-1)
    
    # VR mapping
    position_gain: float = 0.8         # linear gain
    orientation_gain: float = 0.8      # rotation gain
    vr_smoothing: float = 0.02         # Reduced smoothing (0=no smoothing, 1=full smoothing, but higher values lead to lag)
    
    # Safety limits
    position_deadzone: float = 0.00001  # Reduced deadzone (meters) for high sensitivity
    orientation_deadzone: float = 0.0001 # Reduced (radians) for high sensitive rotation
    max_position_offset: float = 1.0   # 60cm from initial robot position (absolute displacement in meters)
    
    # Networking - CRITICAL: Franky loop runs at 1000Hz, VR input at 50Hz
    udp_port: int = 8888
    # The vr_update_rate_hz below is for context only. The network_loop will process
    # data at whatever rate the VR client sends it.
    # The Franky control loop will run at 1kHz regardless.
    # update_rate_hz: float = 1000.0 # This parameter is no longer directly used for motion initiation pacing

    # Safety force thresholds (N for translational, Nm for rotational)
    safety_force_z_threshold: float = 30.0 # Example: Stop if Z-force > 30N


@dataclass
class ErrorRecoveryParams:
    """Error recovery configuration"""
    max_recovery_attempts: int = 3
    recovery_delay_sec: float = 1.0
    enable_auto_recovery: bool = True
    require_manual_confirmation: bool = True


class VRTeleoperation:
    """Main VR teleoperation controller using franky"""
    
    def __init__(self, robot_ip: str):
        self.robot_ip = robot_ip
        self.params = VRControlParams()
        self.recovery_params = ErrorRecoveryParams()
        
        # State variables
        self.running: bool = False
        self.vr_initialized: bool = False
        self.should_stop: bool = False
        
        # VR data
        self.current_vr_command: VRCommand = VRCommand()
        self.vr_command_lock: threading.Lock = threading.Lock()
        # To track if a new VR command has been processed by the control loop
        self._last_processed_vr_timestamp: float = 0.0 
        
        # Reference frames (set during initialization)
        self.initial_robot_pose: Optional[Affine] = None
        self.initial_robot_position: Optional[np.ndarray] = None  # Extracted position as numpy array
        self.initial_robot_orientation: Optional[Rotation] = None  # Extracted orientation as scipy Rotation
        self.initial_vr_position: Optional[np.ndarray] = None
        self.initial_vr_orientation: Optional[Rotation] = None
        
        # Filtered VR state for smoothing
        self.filtered_vr_position: np.ndarray = np.array([0.0, 0.0, 0.0])
        self.filtered_vr_orientation: Rotation = Rotation.from_quat([0, 0, 0, 1])
        
        # Networking
        self.udp_socket: Optional[socket.socket] = None
        self.network_thread: Optional[threading.Thread] = None
        
        # Robot connection
        self.robot: Optional[Robot] = None
        
        # Signal handling
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print(f"\nReceived signal {signum}. Shutting down gracefully...")
        self.stop()

    def setup_networking(self):
        """Initialize UDP server for VR pose data"""
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.bind(('', self.params.udp_port))
            self.udp_socket.settimeout(0.1)  # Non-blocking with timeout
            print(f"UDP server listening on port {self.params.udp_port} for VR pose data")
        except Exception as e:
            raise RuntimeError(f"Failed to setup networking: {e}")

    def network_loop(self):
        """Network thread to receive VR pose data at its native rate (e.g., 50Hz)"""
        while self.running:
            try:
                data, addr = self.udp_socket.recvfrom(1024)
                message = data.decode('utf-8').strip()
                
                # Parse VR command: "pos_x pos_y pos_z quat_x quat_y quat_z quat_w"
                parts = message.split()
                if len(parts) == 7:
                    cmd = VRCommand(
                        pos_x=float(parts[0]), pos_y=float(parts[1]), pos_z=float(parts[2]),
                        quat_x=float(parts[3]), quat_y=float(parts[4]), 
                        quat_z=float(parts[5]), quat_w=float(parts[6]),
                        has_valid_data=True,
                        timestamp=time.time() # Add current timestamp for uniqueness
                    )
                    
                    with self.vr_command_lock:
                        # Only update if this is a newer command
                        if cmd.timestamp > self.current_vr_command.timestamp:
                            self.current_vr_command = cmd
                        
                            # Initialize VR reference on first valid data (and if robot init is done)
                            if not self.vr_initialized and \
                               self.initial_robot_position is not None and \
                               self.initial_robot_orientation is not None:
                                
                                self.initial_vr_position = np.array([cmd.pos_x, cmd.pos_y, cmd.pos_z])
                                self.initial_vr_orientation = Rotation.from_quat([
                                    cmd.quat_x, cmd.quat_y, cmd.quat_z, cmd.quat_w
                                ])
                                self.filtered_vr_position = self.initial_vr_position.copy()
                                # Rotation objects are immutable, so direct assignment is fine
                                self.filtered_vr_orientation = self.initial_vr_orientation 
                                
                                self.vr_initialized = True
                                print("✓ VR reference pose initialized!")
                                print(f"  Initial VR position: {self.initial_vr_position}")
                                print(f"  Initial robot position: {self.initial_robot_position}")
                            
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"Network error in network_loop: {e}")

    def compute_target_pose(self) -> Optional[Affine]:
        """Compute target robot pose from current VR data"""
        with self.vr_command_lock:
            cmd = self.current_vr_command
            
        if not cmd.has_valid_data or not self.vr_initialized or \
           self.initial_robot_position is None or self.initial_robot_orientation is None:
            return None
            
        # Current VR pose
        vr_pos = np.array([cmd.pos_x, cmd.pos_y, cmd.pos_z])
        vr_quat = Rotation.from_quat([cmd.quat_x, cmd.quat_y, cmd.quat_z, cmd.quat_w])
        
        # Apply smoothing to reduce jitter (Exponential Moving Average)
        # alpha is the weight given to the *new* value, (1-alpha) to the *old* filtered value
        alpha = self.params.vr_smoothing 
        self.filtered_vr_position = (alpha * vr_pos + (1.0 - alpha) * self.filtered_vr_position)
        
        # FIX FOR ROTATION SMOOTHING:
        # Calculate the rotation from the current filtered orientation to the new raw VR orientation.
        # Then, apply a fraction (alpha) of this rotation.
        # This is a common way to do exponential smoothing for rotations.
        delta_rotation = vr_quat * self.filtered_vr_orientation.inv()
        
        # Convert delta_rotation to an axis-angle vector
        delta_rotvec = delta_rotation.as_rotvec()
        
        # Apply the smoothing factor to the rotation vector
        smoothed_delta_rotvec = alpha * delta_rotvec
        
        # Convert back to a Rotation object and apply to the filtered orientation
        self.filtered_vr_orientation = Rotation.from_rotvec(smoothed_delta_rotvec) * self.filtered_vr_orientation
        
        # Calculate deltas from initial VR pose
        vr_pos_delta = self.filtered_vr_position - self.initial_vr_position
        
        # FIX FOR ORIENTATION DRIFT:
        # Calculate vr_orientation_delta relative to the INITIAL VR controller orientation,
        # not the initial robot orientation.
        vr_orientation_delta = self.filtered_vr_orientation * self.initial_vr_orientation.inv()
        
        # Apply deadzones
        if np.linalg.norm(vr_pos_delta) < self.params.position_deadzone:
            vr_pos_delta = np.zeros(3)
            
        # For orientation deadzone, use the angle of rotation
        rotation_angle_rad = np.linalg.norm(vr_orientation_delta.as_rotvec())
        if rotation_angle_rad < self.params.orientation_deadzone:
            vr_orientation_delta = Rotation.identity()
        
        # Apply workspace limits on position offset
        current_offset_magnitude = np.linalg.norm(vr_pos_delta)
        if current_offset_magnitude > self.params.max_position_offset:
            vr_pos_delta = (vr_pos_delta / current_offset_magnitude) * \
                           self.params.max_position_offset
        
        # Compute final target robot pose
        target_position = (self.initial_robot_position + 
                           self.params.position_gain * vr_pos_delta)
        target_orientation = vr_orientation_delta * self.initial_robot_orientation
        
        return Affine(target_position.tolist(), target_orientation.as_quat().tolist())


    def setup_robot_safety(self):
        """Configure robot safety parameters"""
        print("Configuring robot safety parameters...")
        
        # Set collision behavior
        # Franky's set_collision_behavior expects lists of 7 for torques, 6 for forces
        lower_torque_thresholds = [100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0]
        upper_torque_thresholds = [100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0]
        lower_force_thresholds = [80.0, 80.0, 80.0, 80.0, 80.0, 80.0] # Fx, Fy, Fz, Tx, Ty, Tz
        upper_force_thresholds = [80.0, 80.0, 80.0, 80.0, 80.0, 80.0]
        
        self.robot.set_collision_behavior(
            lower_torque_thresholds,
            upper_torque_thresholds, 
            lower_force_thresholds,
            upper_force_thresholds
        )
        
        # Set cartesian impedance for smooth motion (example values)
        # These affect how "stiff" the robot feels. Higher values mean stiffer.
        self.robot.set_cartesian_impedance([1000.0, 1000.0, 1000.0, 100.0, 100.0, 100.0]) # N/m for position, Nm/rad for orientation
        
        # Set robot's default relative dynamics factors for all motions.
        # These are the *maximum* factors. Individual motions can override.
        self.robot.relative_dynamics_factor = RelativeDynamicsFactor(
            velocity=self.params.velocity_factor,
            acceleration=self.params.acceleration_factor, 
            jerk=self.params.jerk_factor
        )
        print("✓ Robot safety parameters configured")

    def move_to_initial_pose(self):
        """Move robot to initial joint configuration"""
        from franky import JointWaypointMotion, JointWaypoint

        print("Moving to initial joint configuration...")
        print("WARNING: Robot will move! Make sure to have the user stop button ready!")
        
        # Allow just Enter for confirmation
        input("Press Enter to continue (or Ctrl+C to exit): ")
        
        # Same initial joint configuration as C++ version
        initial_joints = [
            math.radians(60.0), math.radians(-55.0), math.radians(-70.0),
            math.radians(-100.0), math.radians(-30.0), math.radians(160.0),
            math.radians(30.0)
        ]
        
        try:
            # Move to initial joints, using Franky's default dynamics for this move
            joint_motion = JointWaypointMotion([JointWaypoint(initial_joints)])
            self.robot.move(joint_motion)
            print("✓ Moved to initial joint configuration")
        except Exception as e:
            print(f"Error moving to initial joint configuration: {e}")
            raise # Re-raise to trigger main error recovery logic

    def attempt_error_recovery(self, exception: Exception, attempt: int) -> bool:
        """Attempt automatic error recovery with manual confirmation"""
        if not self.recovery_params.enable_auto_recovery:
            return False
            
        print("\n" + "="*50)
        print("🚨 ROBOT ERROR DETECTED!")
        print(f"Error: {exception}")
        print(f"Recovery attempt {attempt}/{self.recovery_params.max_recovery_attempts}")
        print("="*50)
        
        if self.recovery_params.require_manual_confirmation:
            print("\nBefore attempting automatic error recovery:")
            print("1. Check robot status and surroundings")
            print("2. Ensure it's safe to proceed")
            # Allow just Enter for confirmation. Any input implies continuation, unless Ctrl+C
            input("3. Press Enter to attempt recovery (or Ctrl+C to exit): ")

        try:
            print("Attempting automatic error recovery...")
            self.robot.recover_from_errors()
            print("✓ Automatic error recovery successful!")
            
            time.sleep(self.recovery_params.recovery_delay_sec)  # Let robot stabilize
            
            # Reconfigure safety parameters (important after recovery)
            self.setup_robot_safety()
            
            # Re-capture initial robot pose as it might have shifted slightly
            current_cartesian_state = self.robot.current_cartesian_state
            self.initial_robot_pose = current_cartesian_state.pose.end_effector_pose
            
            # Extract position (translation) directly
            self.initial_robot_position = np.array(self.initial_robot_pose.translation)
            
            # Extract rotation from the 4x4 homogeneous matrix (column-major)
            robot_transform_matrix = np.array(self.initial_robot_pose.matrix).reshape(4, 4, order='F') 
            self.initial_robot_orientation = Rotation.from_matrix(robot_transform_matrix[:3, :3])
            
            # Rotation objects are immutable, direct assignment is fine
            # Re-initialize filtered VR orientation from initial VR orientation after recovery
            self.filtered_vr_orientation = self.initial_vr_orientation 
            
            print("✓ Robot ready to continue operation")
            return True
            
        except Exception as recovery_error:
            print(f"✗ Error recovery failed: {recovery_error}")
            return False

    def vr_control_loop(self):
        """
        Main VR control loop. Runs at 1kHz, processing VR data at its native rate (e.g., 50Hz)
        and using Franky's Ruckig for smooth interpolation between VR updates.
        """
        print("🎮 Starting VR control loop...")
        print("Move your VR controller to control the robot!")
        print("Press Ctrl+C to stop")
        
        motion_active = False # Track if a motion has been sent and is still active
        
        try:
            while self.running and not self.should_stop:
                # Synchronize to the robot's 1kHz control cycle
                # This call will block until the next 1ms FCI cycle is ready to read.
                # It also acts as the primary mechanism to receive robot state updates
                # and trigger any stored exceptions from asynchronous motions.
                current_robot_state = self.robot.state 
                
                # --- Check if a NEW VR command has arrived that hasn't been processed ---
                # Acquire lock briefly to check and update the _last_processed_vr_timestamp
                # This makes it robust to your 50Hz input
                should_update_motion = False
                current_vr_data: Optional[VRCommand] = None

                with self.vr_command_lock:
                    if self.current_vr_command.has_valid_data and \
                       self.current_vr_command.timestamp > self._last_processed_vr_timestamp:
                        should_update_motion = True
                        current_vr_data = self.current_vr_command # Get the latest command
                        self._last_processed_vr_timestamp = current_vr_data.timestamp # Mark it as processed

                if should_update_motion:
                    # Compute target pose based on the NEW VR command
                    target_pose = self.compute_target_pose() 
                    
                    if target_pose is not None:
                        # Create a NEW motion to the NEW target pose.
                        # Ruckig will plan a smooth trajectory from the robot's
                        # *current state* to this new `target_pose`.
                        motion = CartesianMotion(
                            target_pose, 
                            ReferenceType.Absolute,
                            RelativeDynamicsFactor(
                                velocity=self.params.velocity_factor,
                                acceleration=self.params.acceleration_factor, 
                                jerk=self.params.jerk_factor
                            )
                        )
                        
                        # Add safety reaction for excessive forces
                        safety_reaction = Reaction(
                            Measure.FORCE_Z > self.params.safety_force_z_threshold,  # Stop if Z-force > threshold
                            CartesianMotion(Affine([0.0, 0.0, 0.0]), ReferenceType.Relative) # Stop motion
                        )
                        motion.add_reaction(safety_reaction)
                        
                        # Send asynchronous motion command.
                        # This will *preempt* any currently running motion
                        # and start a new Ruckig trajectory.
                        self.robot.move(motion, asynchronous=True)
                        motion_active = True
                    else:
                        # If VR data became invalid or target_pose calculation failed for the new command
                        if motion_active: # Only stop if it was previously active
                            print("VR data invalid or target_pose calculation failed, stopping motion.")
                            from franky import CartesianStopMotion
                            self.robot.move(CartesianStopMotion(), asynchronous=True)
                            motion_active = False

                # If `should_update_motion` is False, it means no new VR command has arrived
                # since the last iteration (i.e., we are in one of the 19ms gaps of the 50Hz update).
                # In this case, the robot continues executing the *current* asynchronous motion.
                # We do NOT send another robot.move() call, as that would interrupt Ruckig's
                # smooth planning.

                # No explicit time.sleep() needed here because `self.robot.state`
                # effectively paces the loop at 1kHz.
                
        except KeyboardInterrupt:
            print("\n🛑 VR control interrupted by user")
        except Exception as e:
            print(f"\n❌ Error in VR control loop: {e}")
            raise # Re-raise to trigger main error recovery logic
        finally:
            # Ensure we join any pending motion and handle exceptions
            if motion_active:
                try:
                    print("🔄 Waiting for final motion to complete...")
                    self.robot.join_motion() # Blocks until motion is finished/stopped
                    print("✓ Final motion completed")
                except Exception as join_error:
                    print(f"⚠️ Error during motion cleanup: {join_error}")
            
            # Always ensure robot is stopped
            try:
                from franky import CartesianStopMotion
                self.robot.move(CartesianStopMotion()) # Synchronous stop
                print("✓ Robot explicitly stopped after loop exit")
            except Exception as stop_error:
                print(f"⚠️ Error stopping robot during cleanup: {stop_error}")


    def run(self):
        """Main run function with error recovery"""
        recovery_attempts = 0
        
        while not self.should_stop:
            try:
                print(f"🤖 Connecting to robot at {self.robot_ip}...")
                self.robot = Robot(self.robot_ip)
                print("✓ Connected to robot")
                
                # Setup and initialization
                self.setup_robot_safety()
                
                # Get initial robot pose using franky's API *before* initial move (for display)
                # and then *after* initial move (for control reference)
                current_cartesian_state = self.robot.current_cartesian_state
                self.initial_robot_pose = current_cartesian_state.pose.end_effector_pose
                
                # Extract position (translation) directly
                self.initial_robot_position = np.array(self.initial_robot_pose.translation)
                
                # Extract rotation from the 4x4 homogeneous matrix (column-major)
                robot_transform_matrix = np.array(self.initial_robot_pose.matrix).reshape(4, 4, order='F') 
                self.initial_robot_orientation = Rotation.from_matrix(robot_transform_matrix[:3, :3])
                
                print("✓ Initial robot pose captured from robot (before move)")
                print(f"  Initial robot position: {self.initial_robot_position}")

                # Move to initial pose only if not recovering from an error
                if recovery_attempts == 0:
                    self.move_to_initial_pose()
                    if self.should_stop: # If user aborted initial move
                        break # Exit main run loop
                else:
                    print("Reinitializing after recovery, skipping initial joint move...")
                    # After recovery, the robot is likely still in a safe state,
                    # so no need to move to initial joints again.
                
                # Re-capture initial robot pose *after* `move_to_initial_pose`
                # This is the primary reference for teleoperation offsets.
                current_cartesian_state = self.robot.current_cartesian_state
                self.initial_robot_pose = current_cartesian_state.pose.end_effector_pose
                
                # Extract position (translation) directly
                self.initial_robot_position = np.array(self.initial_robot_pose.translation)
                
                # Extract rotation from the 4x4 homogeneous matrix (column-major)
                robot_transform_matrix = np.array(self.initial_robot_pose.matrix).reshape(4, 4, order='F') 
                self.initial_robot_orientation = Rotation.from_matrix(robot_transform_matrix[:3, :3])
                
                print("✓ Initial robot pose captured from robot (for control reference)")
                print(f"  Final initial robot position: {self.initial_robot_position}")

                # Start networking
                self.setup_networking()
                self.running = True
                self.network_thread = threading.Thread(target=self.network_loop, daemon=True) # daemon thread
                self.network_thread.start()
                print("✓ VR networking started")
                
                # Wait for VR initialization
                print("⏳ Waiting for VR data to initialize base pose...")
                # The VR initialization now also depends on initial_robot_position/orientation being set
                while not self.vr_initialized and self.running and not self.should_stop:
                    time.sleep(0.1)
                
                if self.vr_initialized and not self.should_stop:
                    print("✓ VR initialized! Starting active control")
                    self.vr_control_loop()
                else:
                    print("VR initialization failed or stopped before control loop started.")
                
                # Successful completion or intentional stop - exit loop
                break
                
            except Exception as e:
                print(f"\n❌ Robot or control loop error: {e}")
                recovery_attempts += 1
                
                # Attempt recovery
                if recovery_attempts <= self.recovery_params.max_recovery_attempts:
                    # Clean up networking before attempting recovery to avoid stale data
                    self.running = False
                    if self.network_thread and self.network_thread.is_alive():
                        self.network_thread.join(timeout=1.0)
                        self.network_thread = None # Reset thread
                    if self.udp_socket:
                        self.udp_socket.close()
                        self.udp_socket = None # Reset socket
                    
                    self._last_processed_vr_timestamp = 0.0 # Reset processed timestamp
                    self.vr_initialized = False # Reset VR init flag
                    
                    if self.attempt_error_recovery(e, recovery_attempts):
                        print("\n🔄 Preparing to restart VR control...")
                        print("Make sure VR system is ready and data is flowing!")
                        # Allow just Enter for confirmation
                        input("Press Enter to re-attempt connection and control: ")
                        
                        continue # Continue main loop to retry connection
                    
                print(f"💀 Maximum recovery attempts reached or recovery aborted. Exiting.")
                break # Exit main run loop
        
        self.cleanup()

    def stop(self):
        """Stop the VR teleoperation system"""
        print("\n🛑 Stopping VR teleoperation...")
        self.should_stop = True
        self.running = False

    def cleanup(self):
        """Clean up resources"""
        print("🧹 Cleaning up...")
        
        self.running = False # Ensure network_loop also terminates
        self.should_stop = True # Ensure main loop also terminates

        # Wait for network thread to finish
        if self.network_thread and self.network_thread.is_alive():
            print("Joining network thread...")
            self.network_thread.join(timeout=2.0)
            if self.network_thread.is_alive():
                print("Warning: Network thread did not terminate gracefully.")
        
        # Close UDP socket
        if self.udp_socket:
            self.udp_socket.close()
            print("✓ UDP socket closed")
        
        # Stop robot motion and join any pending async motions
        if self.robot:
            try:
                print("Attempting to join any active robot motion...")
                self.robot.join_motion() # Blocks until motion is finished/stopped
                print("✓ Any pending motion joined.")
            except Exception as join_error:
                print(f"⚠️ Warning during final motion join: {join_error}")
            
            try:
                from franky import CartesianStopMotion
                self.robot.move(CartesianStopMotion()) # Send a final synchronous stop
                print("✓ Robot explicitly stopped.")
            except Exception as stop_error:
                print(f"⚠️ Warning during final robot stop: {stop_error}")
        
        print("✓ Cleanup complete")


def main():
    """Main function"""
    if len(sys.argv) != 2:
        print("Usage: python vr_teleoperation.py <robot-hostname>")
        sys.exit(1)
    
    robot_ip = sys.argv[1]
    
    controller = None # Initialize to None
    try:
        controller = VRTeleoperation(robot_ip)
        controller.run()
    except Exception as e:
        print(f"Fatal error encountered in main: {e}")
        # Cleanup will be called by the `finally` block if controller was instantiated
    finally:
        if controller:
            controller.cleanup() # Ensure cleanup is always called

if __name__ == "__main__":
    main()