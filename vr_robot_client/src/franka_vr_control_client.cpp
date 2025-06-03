// vr_franka_control_client.cpp - VR pose control for Franka robot
// Based on modern libfranka external control loop API
#include <cmath>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <array>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <franka/active_control.h>
#include <franka/active_motion_generator.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"

struct VRCommand {
    double linear_x = 0.0;
    double linear_y = 0.0; 
    double linear_z = 0.0;
    double angular_x = 0.0;
    double angular_y = 0.0;
    double angular_z = 0.0;
    bool emergency_stop = false;
    bool reset_pose = false;
};

class VRFrankaController {
private:
    std::atomic<bool> running_{true};
    std::atomic<bool> emergency_stop_{false};
    VRCommand current_command_;
    std::mutex command_mutex_;
    
    int server_socket_;
    const int PORT = 8888;
    
    // Movement limits - more conservative for VR control
    const double MAX_LINEAR_VEL = 0.03;   // 3cm/s max
    const double MAX_ANGULAR_VEL = 0.03;  // 0.03 rad/s max
    const double POSE_SMOOTHING = 0.05;   // Very aggressive smoothing for pose control
    
    // Pose tracking
    std::array<double, 16> initial_pose_;
    std::array<double, 16> current_target_pose_;
    bool pose_initialized_ = false;
    
    // Smoothed pose for continuity
    std::array<double, 6> smoothed_pose_delta_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> prev_pose_delta_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    
public:
    VRFrankaController() {
        setupNetworking();
    }
    
    ~VRFrankaController() {
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
        
        std::cout << "UDP server listening on port " << PORT << std::endl;
    }
    
    void networkThread() {
        char buffer[1024];
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        
        while (running_) {
            ssize_t bytes_received = recvfrom(server_socket_, buffer, sizeof(buffer), 0,
                                            (struct sockaddr*)&client_addr, &client_len);
            
            if (bytes_received > 0) {
                VRCommand cmd;
                if (sscanf(buffer, "%lf %lf %lf %lf %lf %lf %d %d",
                          &cmd.linear_x, &cmd.linear_y, &cmd.linear_z,
                          &cmd.angular_x, &cmd.angular_y, &cmd.angular_z,
                          (int*)&cmd.emergency_stop, (int*)&cmd.reset_pose) == 8) {
                    
                    std::lock_guard<std::mutex> lock(command_mutex_);
                    current_command_ = cmd;
                    
                    if (cmd.emergency_stop) {
                        emergency_stop_ = true;
                        std::cout << "Emergency stop received!" << std::endl;
                    }
                    
                    // Debug output for VR commands
                    static int debug_counter = 0;
                    debug_counter++;
                    if (debug_counter % 25 == 0) { // Every 0.5 seconds at 50Hz
                        if (std::abs(cmd.linear_x) > 0.001 || std::abs(cmd.linear_y) > 0.001 || std::abs(cmd.linear_z) > 0.001) {
                            std::cout << "VR Pose: lin=[" << cmd.linear_x << "," << cmd.linear_y << "," << cmd.linear_z 
                                      << "] ang=[" << cmd.angular_x << "," << cmd.angular_y << "," << cmd.angular_z << "]" << std::endl;
                        }
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    std::array<double, 16> composePose(const std::array<double, 16>& base_pose, 
                                      const std::array<double, 6>& pose_delta) {
        std::array<double, 16> new_pose = base_pose;
        
        // Apply translational delta
        new_pose[12] += pose_delta[0];  // x
        new_pose[13] += pose_delta[1];  // y  
        new_pose[14] += pose_delta[2];  // z
        
        // For orientation, we need to apply rotational delta properly
        // This is a simplified approach - in production you'd want proper SE(3) composition
        // For small rotations, we can approximate by modifying the rotation matrix
        
        return new_pose;
    }
    
    bool isValidPose(const std::array<double, 16>& pose) {
        // Check for NaN or inf values
        for (int i = 0; i < 16; i++) {
            if (!std::isfinite(pose[i])) {
                std::cout << "Invalid pose: non-finite value at index " << i << std::endl;
                return false;
            }
        }
        
        // Check position is reasonable (within 1m of initial position)
        if (pose_initialized_) {
            double dx = pose[12] - initial_pose_[12];
            double dy = pose[13] - initial_pose_[13];
            double dz = pose[14] - initial_pose_[14];
            double distance = sqrt(dx*dx + dy*dy + dz*dz);
            
            if (distance > 0.5) { // 50cm max movement
                std::cout << "Invalid pose: too far from initial position (" << distance << "m)" << std::endl;
                return false;
            }
        }
        
        return true;
    }
    
    void run(const std::string& robot_ip) {
        try {
            std::cout << "Connecting to robot at " << robot_ip << std::endl;
            franka::Robot robot(robot_ip);
            setDefaultBehavior(robot);
            
            // First move to initial joint configuration
            std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            MotionGenerator motion_generator(0.5, q_goal);
            std::cout << "WARNING: Robot will move based on VR input!" << std::endl
                      << "Put on your VR headset and make sure to have the emergency stop ready!" << std::endl
                      << "Press Enter to continue..." << std::endl;
            std::cin.ignore();
            robot.control(motion_generator);
            std::cout << "Finished moving to initial joint configuration." << std::endl;
            
            // Set collision behavior - more permissive for smooth VR control
            robot.setCollisionBehavior(
                {{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}}, {{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}},
                {{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}}, {{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}},
                {{25.0, 25.0, 25.0, 25.0, 25.0, 25.0}}, {{25.0, 25.0, 25.0, 25.0, 25.0, 25.0}},
                {{25.0, 25.0, 25.0, 25.0, 25.0, 25.0}}, {{25.0, 25.0, 25.0, 25.0, 25.0, 25.0}});
            
            std::cout << "Starting network thread..." << std::endl;
            std::thread network_thread(&VRFrankaController::networkThread, this);
            
            std::cout << "Starting VR pose control. Move your VR hand to control robot." << std::endl;
            std::cout << "The robot will follow your hand movements with scaling and smoothing." << std::endl;
            
            // Create pose control callback using external control loop
            auto callback_control = [this](const franka::RobotState& robot_state,
                                          franka::Duration period) -> franka::CartesianPose {
                
                // Initialize pose on first call
                if (!pose_initialized_) {
                    initial_pose_ = robot_state.O_T_EE_c;
                    current_target_pose_ = initial_pose_;
                    pose_initialized_ = true;
                    std::cout << "Initial robot pose captured. VR control active!" << std::endl;
                    return current_target_pose_;
                }
                
                // Check for emergency stop
                if (emergency_stop_) {
                    std::cout << "Emergency stop activated!" << std::endl;
                    return franka::MotionFinished(current_target_pose_);
                }
                
                // Get current VR command
                VRCommand cmd;
                {
                    std::lock_guard<std::mutex> lock(command_mutex_);
                    cmd = current_command_;
                }
                
                // Convert VR velocities to pose increments
                double dt = period.toSec();
                std::array<double, 6> target_pose_delta = {{
                    cmd.linear_x * MAX_LINEAR_VEL * dt,   // x
                    cmd.linear_y * MAX_LINEAR_VEL * dt,   // y
                    cmd.linear_z * MAX_LINEAR_VEL * dt,   // z
                    cmd.angular_x * MAX_ANGULAR_VEL * dt, // rx
                    cmd.angular_y * MAX_ANGULAR_VEL * dt, // ry
                    cmd.angular_z * MAX_ANGULAR_VEL * dt  // rz
                }};
                
                // Apply smoothing to pose deltas
                for (int i = 0; i < 6; i++) {
                    smoothed_pose_delta_[i] += POSE_SMOOTHING * (target_pose_delta[i] - smoothed_pose_delta_[i]);
                }
                
                // Limit delta changes to prevent acceleration discontinuities
                const double MAX_DELTA_CHANGE = 0.0005; // 0.5mm or 0.5mrad per cycle
                for (int i = 0; i < 6; i++) {
                    double delta_change = smoothed_pose_delta_[i] - prev_pose_delta_[i];
                    if (delta_change > MAX_DELTA_CHANGE) {
                        smoothed_pose_delta_[i] = prev_pose_delta_[i] + MAX_DELTA_CHANGE;
                    } else if (delta_change < -MAX_DELTA_CHANGE) {
                        smoothed_pose_delta_[i] = prev_pose_delta_[i] - MAX_DELTA_CHANGE;
                    }
                    prev_pose_delta_[i] = smoothed_pose_delta_[i];
                }
                
                // Apply smoothed pose delta to current robot pose
                current_target_pose_ = composePose(robot_state.O_T_EE_c, smoothed_pose_delta_);
                
                // Validate pose
                if (!isValidPose(current_target_pose_)) {
                    std::cout << "Invalid target pose! Resetting to current." << std::endl;
                    current_target_pose_ = robot_state.O_T_EE_c;
                    // Reset smoothing
                    std::fill(smoothed_pose_delta_.begin(), smoothed_pose_delta_.end(), 0.0);
                    std::fill(prev_pose_delta_.begin(), prev_pose_delta_.end(), 0.0);
                }
                
                return current_target_pose_;
            };
            
            // Control loop with automatic restart after errors
            while (!emergency_stop_) {
                try {
                    // Start external pose control loop
                    bool motion_finished = false;
                    auto active_control = robot.startCartesianPoseControl(
                        research_interface::robot::Move::ControllerMode::kJointImpedance);
                    
                    while (!motion_finished && !emergency_stop_) {
                        auto read_once_return = active_control->readOnce();
                        auto robot_state = read_once_return.first;
                        auto duration = read_once_return.second;
                        auto cartesian_pose = callback_control(robot_state, duration);
                        motion_finished = cartesian_pose.motion_finished;
                        active_control->writeOnce(cartesian_pose);
                    }
                    
                    if (motion_finished) {
                        std::cout << "VR control finished normally." << std::endl;
                        break;
                    }
                    
                } catch (const franka::ControlException& e) {
                    std::cout << "Control exception: " << e.what() << std::endl;
                    std::cout << "Running error recovery..." << std::endl;
                    robot.automaticErrorRecovery();
                    
                    // Reset smoothing after error recovery
                    std::fill(smoothed_pose_delta_.begin(), smoothed_pose_delta_.end(), 0.0);
                    // DON'T reset prev_pose_delta_ to avoid discontinuity
                    
                    std::cout << "Restarting VR control loop..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
            }
            
            std::cout << "VR control finished." << std::endl;
            running_ = false;
            network_thread.join();
            
        } catch (const franka::Exception& e) {
            std::cout << "Franka exception: " << e.what() << std::endl;
            running_ = false;
        } catch (const std::exception& e) {
            std::cout << "Standard exception: " << e.what() << std::endl;
            running_ = false;
        }
    }
};

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    
    try {
        VRFrankaController controller;
        controller.run(argv[1]);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}