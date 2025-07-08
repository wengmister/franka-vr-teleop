// VR-Based Cartesian Teleoperation - Pose control from VR input
#include <cmath>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <array>
#include <chrono>
#include <deque>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <Eigen/Dense>
#include "examples_common.h"

struct VRCommand {
    double pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;
    double quat_x = 0.0, quat_y = 0.0, quat_z = 0.0, quat_w = 1.0;
    bool has_valid_data = false;
};

// Simple trajectory point with guaranteed smooth derivatives
struct TrajectoryPoint {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d angular_acceleration;
    double time;
};

class VRCartesianController {
private:
    std::atomic<bool> running_{true};
    std::atomic<bool> emergency_stop_{false};
    VRCommand current_vr_command_;
    std::mutex command_mutex_;
    
    int server_socket_;
    const int PORT = 8888;
    
    // Trajectory parameters - designed for smooth VR pose following
    struct TrajParams {
        double max_velocity = 0.06;           // Reduced from 0.05 (3cm/s)
        double max_acceleration = 0.02;      // Reduced from 0.02 (1.5cm/s²)
        double max_jerk = 0.05;               // Reduced from 0.1 (5cm/s³)
        double max_angular_velocity = 0.3;   // Reduced from 0.2 (0.15 rad/s)
        double max_angular_acceleration = 0.02; // Reduced from 0.02 (0.015 rad/s²)
        double max_angular_jerk = 0.05;        // Reduced from 0.2 (0.1 rad/s³)
        
        // VR pose following parameters - reduced for stability
        double position_tracking_gain = 0.1;   // Reduced from 0.2
        double orientation_tracking_gain = 0.1; // Reduced from 0.2
        double smoothing_factor = 0.8;         // Increased from 0.05 for more smoothing
        
        // Damping factors for stability
        double position_damping = 0.1;         // New: velocity damping
        double angular_damping = 0.1;          // New: angular velocity damping
        
        // Deadzone for VR input
        double position_deadzone = 0.01;       // 10mm
        double orientation_deadzone = 0.1;    // ~0.3 degrees
        
        // Error limits to prevent runaway
        double max_position_error = 0.1;       // 10cm max position error
        double max_orientation_error = 0.5;    // ~30 degrees max orientation error
    } params_;
    
    // Current trajectory state
    TrajectoryPoint current_point_;
    TrajectoryPoint target_point_;
    
    // VR target filtering
    Eigen::Vector3d filtered_vr_position_{0, 0, 0};
    Eigen::Quaterniond filtered_vr_orientation_{1, 0, 0, 0};
    
    // Trajectory history for smooth derivatives
    std::deque<TrajectoryPoint> trajectory_history_;
    
    // Initial poses
    Eigen::Affine3d initial_robot_pose_;
    Eigen::Vector3d initial_vr_position_{0, 0, 0};
    Eigen::Quaterniond initial_vr_orientation_{1, 0, 0, 0};
    bool vr_initialized_ = false;
    
public:
    VRCartesianController() {
        setupNetworking();
        initializeTrajectoryPoint(current_point_);
        initializeTrajectoryPoint(target_point_);
    }
    
    ~VRCartesianController() {
        running_ = false;
        close(server_socket_);
    }
    
    void setupNetworking() {
        server_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (server_socket_ < 0) {
            throw std::runtime_error("Failed to create socket");
        }
        
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(PORT);
        
        if (bind(server_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            throw std::runtime_error("Failed to bind socket");
        }
        
        int flags = fcntl(server_socket_, F_GETFL, 0);
        fcntl(server_socket_, F_SETFL, flags | O_NONBLOCK);
        
        std::cout << "UDP server listening on port " << PORT << " for VR pose data" << std::endl;
    }
    
    void networkThread() {
        char buffer[1024];
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        
        while (running_) {
            ssize_t bytes_received = recvfrom(server_socket_, buffer, sizeof(buffer), 0,
                                            (struct sockaddr*)&client_addr, &client_len);
            
            if (bytes_received > 0) {
                buffer[bytes_received] = '\0';
                
                VRCommand cmd;
                // Parse VR pose: "pos_x pos_y pos_z quat_x quat_y quat_z quat_w"
                int parsed_count = sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf",
                          &cmd.pos_x, &cmd.pos_y, &cmd.pos_z, 
                          &cmd.quat_x, &cmd.quat_y, &cmd.quat_z, &cmd.quat_w);
                
                if (parsed_count == 7) {
                    cmd.has_valid_data = true;
                    
                    std::lock_guard<std::mutex> lock(command_mutex_);
                    current_vr_command_ = cmd;
                    
                    // Initialize VR reference on first valid data
                    if (!vr_initialized_) {
                        initial_vr_position_ = Eigen::Vector3d(cmd.pos_x, cmd.pos_y, cmd.pos_z);
                        initial_vr_orientation_ = Eigen::Quaterniond(cmd.quat_w, cmd.quat_x, cmd.quat_y, cmd.quat_z);
                        initial_vr_orientation_.normalize();
                        
                        filtered_vr_position_ = initial_vr_position_;
                        filtered_vr_orientation_ = initial_vr_orientation_;
                        
                        vr_initialized_ = true;
                        std::cout << "VR reference pose initialized!" << std::endl;
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
private:
    void initializeTrajectoryPoint(TrajectoryPoint& point) {
        point.position.setZero();
        point.velocity.setZero();
        point.acceleration.setZero();
        point.orientation.setIdentity();
        point.angular_velocity.setZero();
        point.angular_acceleration.setZero();
        point.time = 0.0;
    }
    
    void processVRInput(const VRCommand& cmd, double dt) {
        if (!cmd.has_valid_data || !vr_initialized_) {
            return;
        }
        
        // Current VR pose
        Eigen::Vector3d vr_position(cmd.pos_x, cmd.pos_y, cmd.pos_z);
        Eigen::Quaterniond vr_orientation(cmd.quat_w, cmd.quat_x, cmd.quat_y, cmd.quat_z);
        vr_orientation.normalize();
        
        // Apply smoothing to VR input (more aggressive smoothing)
        double alpha = (1.0 - params_.smoothing_factor);
        
        // Smooth position
        filtered_vr_position_ = params_.smoothing_factor * filtered_vr_position_ + 
                               alpha * vr_position;
        
        // Smooth orientation using SLERP
        filtered_vr_orientation_ = filtered_vr_orientation_.slerp(alpha, vr_orientation);
        
        // Calculate VR deltas from initial pose
        Eigen::Vector3d vr_position_delta = filtered_vr_position_ - initial_vr_position_;
        Eigen::Quaterniond vr_orientation_delta = filtered_vr_orientation_ * initial_vr_orientation_.inverse();
        
        // Apply deadzone
        if (vr_position_delta.norm() < params_.position_deadzone) {
            vr_position_delta.setZero();
        }
        
        // For orientation deadzone, check the rotation angle
        double rotation_angle = 2.0 * std::acos(std::abs(vr_orientation_delta.w()));
        if (rotation_angle < params_.orientation_deadzone) {
            vr_orientation_delta.setIdentity();
        }
        
        // Calculate target robot pose (initial robot pose + VR delta)
        target_point_.position = initial_robot_pose_.translation() + vr_position_delta;
        
        // FIXED: Correct quaternion multiplication order
        // Apply VR delta rotation to the initial robot orientation
        target_point_.orientation = vr_orientation_delta * Eigen::Quaterniond(initial_robot_pose_.rotation());
        target_point_.orientation.normalize();
        
        // Apply error limits to prevent runaway behavior
        Eigen::Vector3d position_error = target_point_.position - current_point_.position;
        if (position_error.norm() > params_.max_position_error) {
            position_error = position_error.normalized() * params_.max_position_error;
            target_point_.position = current_point_.position + position_error;
        }
        
        // Limit orientation error
        Eigen::Quaterniond orientation_error = target_point_.orientation * current_point_.orientation.inverse();
        double orientation_angle = 2.0 * std::acos(std::abs(orientation_error.w()));
        if (orientation_angle > params_.max_orientation_error) {
            // Scale down the rotation to the maximum allowed
            Eigen::AngleAxisd angle_axis(orientation_error);
            double scale_factor = params_.max_orientation_error / orientation_angle;
            Eigen::AngleAxisd scaled_rotation(angle_axis.angle() * scale_factor, angle_axis.axis());
            target_point_.orientation = current_point_.orientation * Eigen::Quaterniond(scaled_rotation);
            target_point_.orientation.normalize();
        }
    }
    
    TrajectoryPoint generateNextTrajectoryPoint(double dt) {
        TrajectoryPoint next_point = current_point_;
        next_point.time += dt;
        
        // Calculate desired velocities to reach target pose
        Eigen::Vector3d position_error = target_point_.position - current_point_.position;
        Eigen::Vector3d desired_velocity = position_error * params_.position_tracking_gain;
        
        // Add damping to desired velocity
        Eigen::Vector3d position_damping = -params_.position_damping * current_point_.velocity;
        desired_velocity += position_damping;
        
        // Calculate desired angular velocity to reach target orientation
        Eigen::Quaterniond orientation_error = target_point_.orientation * current_point_.orientation.inverse();
        orientation_error.normalize();
        
        // IMPROVED: Better angular velocity calculation using angle-axis
        Eigen::Vector3d desired_angular_velocity = Eigen::Vector3d::Zero();
        if (std::abs(orientation_error.w()) < 1.0) {
            Eigen::AngleAxisd angle_axis(orientation_error);
            double angle = angle_axis.angle();
            if (angle > 1e-6) {
                // Ensure we take the shortest path
                if (angle > M_PI) {
                    angle = 2 * M_PI - angle;
                    angle_axis = Eigen::AngleAxisd(-angle, angle_axis.axis());
                }
                desired_angular_velocity = angle_axis.axis() * angle * params_.orientation_tracking_gain;
            }
        }
        
        // Add angular damping
        Eigen::Vector3d angular_damping = -params_.angular_damping * current_point_.angular_velocity;
        desired_angular_velocity += angular_damping;
        
        // Generate smooth trajectory with acceleration/jerk limits
        for (int i = 0; i < 3; i++) {
            // Linear motion
            double vel_error = desired_velocity[i] - current_point_.velocity[i];
            double desired_accel = vel_error / dt;
            double accel_error = desired_accel - current_point_.acceleration[i];
            
            // Limit jerk
            double jerk = std::max(-params_.max_jerk, 
                         std::min(params_.max_jerk, accel_error / dt));
            
            // Update acceleration with jerk limit
            next_point.acceleration[i] = current_point_.acceleration[i] + jerk * dt;
            
            // Limit acceleration magnitude
            next_point.acceleration[i] = std::max(-params_.max_acceleration,
                                        std::min(params_.max_acceleration, next_point.acceleration[i]));
            
            // Update velocity with acceleration
            next_point.velocity[i] = current_point_.velocity[i] + next_point.acceleration[i] * dt;
            
            // Limit velocity magnitude
            next_point.velocity[i] = std::max(-params_.max_velocity,
                                    std::min(params_.max_velocity, next_point.velocity[i]));
            
            // Update position with velocity
            next_point.position[i] = current_point_.position[i] + next_point.velocity[i] * dt;
            
            // Angular motion
            double angular_vel_error = desired_angular_velocity[i] - current_point_.angular_velocity[i];
            double desired_angular_accel = angular_vel_error / dt;
            double angular_accel_error = desired_angular_accel - current_point_.angular_acceleration[i];
            
            double angular_jerk = std::max(-params_.max_angular_jerk,
                                 std::min(params_.max_angular_jerk, angular_accel_error / dt));
            
            next_point.angular_acceleration[i] = current_point_.angular_acceleration[i] + angular_jerk * dt;
            next_point.angular_acceleration[i] = std::max(-params_.max_angular_acceleration,
                                                std::min(params_.max_angular_acceleration, next_point.angular_acceleration[i]));
            
            next_point.angular_velocity[i] = current_point_.angular_velocity[i] + next_point.angular_acceleration[i] * dt;
            next_point.angular_velocity[i] = std::max(-params_.max_angular_velocity,
                                            std::min(params_.max_angular_velocity, next_point.angular_velocity[i]));
        }
        
        // Update orientation using angular velocity
        if (next_point.angular_velocity.norm() > 1e-8) {
            double angle = next_point.angular_velocity.norm() * dt;
            Eigen::Vector3d axis = next_point.angular_velocity.normalized();
            Eigen::Quaterniond delta_quat(Eigen::AngleAxisd(angle, axis));
            next_point.orientation = current_point_.orientation * delta_quat;
            next_point.orientation.normalize();
        } else {
            next_point.orientation = current_point_.orientation;
        }
        
        // Workspace enforcement with smooth boundaries
        enforceWorkspaceLimits(next_point);
        
        return next_point;
    }
    
    void enforceWorkspaceLimits(TrajectoryPoint& point) {
        Eigen::Vector3d relative_pos = point.position - initial_robot_pose_.translation();
        Eigen::Vector3d workspace_min{-0.25, -0.25, -0.25};
        Eigen::Vector3d workspace_max{0.25, 0.25, 0.25};
        
        bool hit_limit = false;
        for (int i = 0; i < 3; i++) {
            if (relative_pos[i] < workspace_min[i]) {
                relative_pos[i] = workspace_min[i];
                point.velocity[i] = std::max(0.0, point.velocity[i]);
                point.acceleration[i] = 0.0;
                hit_limit = true;
            } else if (relative_pos[i] > workspace_max[i]) {
                relative_pos[i] = workspace_max[i];
                point.velocity[i] = std::min(0.0, point.velocity[i]);
                point.acceleration[i] = 0.0;
                hit_limit = true;
            }
        }
        
        if (hit_limit) {
            point.position = initial_robot_pose_.translation() + relative_pos;
            static int limit_warn_count = 0;
            limit_warn_count++;
            if (limit_warn_count % 1000 == 0) {
                std::cout << "Workspace limit enforced" << std::endl;
            }
        }
    }
    
    franka::CartesianPose trajectoryPointToCartesianPose(const TrajectoryPoint& point) {
        Eigen::Affine3d pose;
        pose.translation() = point.position;
        pose.linear() = point.orientation.toRotationMatrix();
        
        std::array<double, 16> pose_array;
        Eigen::Map<Eigen::Matrix4d>(pose_array.data()) = pose.matrix();
        return franka::CartesianPose(pose_array);
    }
    
public:
    void run(const std::string& robot_ip) {
        try {
            std::cout << "Connecting to robot at " << robot_ip << std::endl;
            franka::Robot robot(robot_ip);
            setDefaultBehavior(robot);
            
            // Move to initial configuration
            // Joint angles: [60, -80, -80, -100, -45, 170, 80] degrees
            std::array<double, 7> q_goal = {{
                60.0 * M_PI / 180.0,   // 60° → 1.047 rad
                -55.0 * M_PI / 180.0,  // -80° → -1.396 rad  
                -70.0 * M_PI / 180.0,  // -80° → -1.396 rad
                -100.0 * M_PI / 180.0, // -100° → -1.745 rad
                -30.0 * M_PI / 180.0,  // -45° → -0.785 rad
                160.0 * M_PI / 180.0,  // 160
                30.0 * M_PI / 180.0    // 40°
            }};
            MotionGenerator motion_generator(0.5, q_goal);
            std::cout << "WARNING: This example will move the robot!" << std::endl
                      << "Press Enter to continue..." << std::endl;
            std::cin.ignore();
            robot.control(motion_generator);
            
            // Configure robot for smooth operation with more conservative settings
            robot.setCollisionBehavior(
                {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}}, {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}},
                {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}}, {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}},
                {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}},
                {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}});
            
            // Higher impedance for more stability
            robot.setCartesianImpedance({{1000, 1000, 1000, 100, 100, 100}});
            
            // Initialize trajectory from current pose
            franka::RobotState state = robot.readOnce();
            initial_robot_pose_ = Eigen::Affine3d(Eigen::Matrix4d::Map(state.O_T_EE_d.data()));
            
            current_point_.position = initial_robot_pose_.translation();
            current_point_.orientation = Eigen::Quaterniond(initial_robot_pose_.rotation());
            
            target_point_.position = current_point_.position;
            target_point_.orientation = current_point_.orientation;
            
            std::thread network_thread(&VRCartesianController::networkThread, this);
            
            std::cout << "Waiting for VR data..." << std::endl;
            while (!vr_initialized_ && running_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            if (vr_initialized_) {
                std::cout << "VR initialized! Starting robot control..." << std::endl;
                this->runVRCartesianTeleop(robot);
            }
            
            running_ = false;
            network_thread.join();
            
        } catch (const franka::Exception& e) {
            std::cout << "Franka exception: " << e.what() << std::endl;
            running_ = false;
        }
    }
    
private:
    void runVRCartesianTeleop(franka::Robot& robot) {
        std::cout << "Starting VR-based Cartesian teleoperation..." << std::endl;
        
        try {
            robot.automaticErrorRecovery();
        } catch (const franka::Exception& e) {
            std::cout << "Error recovery: " << e.what() << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        int iteration_count = 0;
        
        auto trajectory_generator = [this, iteration_count = 0]
                                   (const franka::RobotState& robot_state, franka::Duration period) mutable -> franka::CartesianPose {
            
            iteration_count++;
            double dt = period.toSec();
            
            if (emergency_stop_) {
                return franka::MotionFinished(trajectoryPointToCartesianPose(current_point_));
            }
            
            // Get VR command
            VRCommand cmd;
            {
                std::lock_guard<std::mutex> lock(command_mutex_);
                cmd = current_vr_command_;
            }
            
            // Only start control after initialization
            if (iteration_count > 10 && dt > 0.0 && dt < 0.01 && vr_initialized_) {
                processVRInput(cmd, dt);
                current_point_ = generateNextTrajectoryPoint(dt);
                
                // More frequent debug output for monitoring
                if (iteration_count % 1000 == 0) {
                    Eigen::Vector3d pos_error = target_point_.position - current_point_.position;
                    double orient_error = target_point_.orientation.angularDistance(current_point_.orientation);
                    
                    std::cout << "VR Teleop: pos=[" << current_point_.position.x() << ", "
                              << current_point_.position.y() << ", " << current_point_.position.z() << "]"
                              << " pos_err=" << pos_error.norm()
                              << " orient_err=" << orient_error
                              << " vel=" << current_point_.velocity.norm() 
                              << " angular_vel=" << current_point_.angular_velocity.norm() << std::endl;
                }
            }
            
            return trajectoryPointToCartesianPose(current_point_);
        };
        
        try {
            robot.control(trajectory_generator);
            std::cout << "VR control finished normally." << std::endl;
        } catch (const franka::ControlException& e) {
            std::cout << "VR control exception: " << e.what() << std::endl;
            std::cout << "Final state: pos=[" << current_point_.position.x() << ", "
                      << current_point_.position.y() << ", " << current_point_.position.z() << "]"
                      << " vel=" << current_point_.velocity.norm() << std::endl;
        }
    }
};

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    
    try {
        VRCartesianController controller;
        controller.run(argv[1]);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}