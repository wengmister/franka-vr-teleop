// Hybrid VR-Based Joint Teleoperation with Multi-Objective IK Optimization
// Combines advanced IK solver with clean architecture and robust error recovery
// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

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
#include <csignal>
#include <limits>
#include <optional>

#include <franka/exception.h>
#include <franka/robot.h>
#include <Eigen/Dense>

#include "examples_common.h"
#include "panda_kinematics.hpp"

struct VRCommand
{
    double pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;
    double quat_x = 0.0, quat_y = 0.0, quat_z = 0.0, quat_w = 1.0;
    bool has_valid_data = false;
};

// Enhanced Custom IK Solver with Multi-Objective Optimization
class EnhancedIKSolver 
{
private:
    struct IKParams {
        double position_tolerance = 1e-4;        // 0.1mm precision
        double orientation_tolerance = 1e-3;     // ~0.057 degrees
        int max_iterations = 100;
        double damping_factor = 0.01;            // Damped least squares
        double manipulability_weight = 0.1;      // Manipulability maximization
        double joint_limit_weight = 0.05;        // Joint limit avoidance
        double config_distance_weight = 0.2;     // Prefer nearby configurations
        double step_size = 0.01;                 // Conservative step size
    } ik_params_;

    // Joint limits for Franka Panda (in radians)
    Eigen::Matrix<double, 7, 1> q_min_;
    Eigen::Matrix<double, 7, 1> q_max_;
    Eigen::Matrix<double, 7, 1> q_neutral_;  // Preferred joint configuration

public:
    EnhancedIKSolver() {
        // Franka Panda joint limits
        q_min_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
        q_max_ << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
        
        // Neutral/preferred configuration
        q_neutral_ << 60.0 * M_PI / 180.0, -55.0 * M_PI / 180.0, -70.0 * M_PI / 180.0,
                     -100.0 * M_PI / 180.0, -30.0 * M_PI / 180.0, 160.0 * M_PI / 180.0,
                     30.0 * M_PI / 180.0;
    }

    // Calculate manipulability index
    double calculateManipulability(const Eigen::Matrix<double, 6, 7>& jacobian) {
        Eigen::Matrix<double, 6, 6> JJT = jacobian * jacobian.transpose();
        return std::sqrt(JJT.determinant());
    }

    // Calculate configuration distance from current joint configuration
    double calculateConfigDistance(const Eigen::Matrix<double, 7, 1>& q_candidate,
                                  const Eigen::Matrix<double, 7, 1>& q_current) {
        return (q_candidate - q_current).squaredNorm();
    }

    // Gradient of configuration distance
    Eigen::Matrix<double, 7, 1> calculateConfigDistanceGradient(
        const Eigen::Matrix<double, 7, 1>& q_candidate,
        const Eigen::Matrix<double, 7, 1>& q_current) {
        return 2.0 * (q_candidate - q_current);
    }

    // Calculate joint limit penalty
    double calculateJointLimitPenalty(const Eigen::Matrix<double, 7, 1>& q) {
        double penalty = 0.0;
        for (int i = 0; i < 7; ++i) {
            double range = q_max_[i] - q_min_[i];
            double center = (q_max_[i] + q_min_[i]) / 2.0;
            double normalized_dist = std::abs(q[i] - center) / (range / 2.0);
            
            // Exponential penalty as we approach limits
            if (normalized_dist > 0.8) {
                penalty += std::exp(10.0 * (normalized_dist - 0.8));
            }
        }
        return penalty;
    }

    // Gradient of joint limit penalty
    Eigen::Matrix<double, 7, 1> calculateJointLimitGradient(const Eigen::Matrix<double, 7, 1>& q) {
        Eigen::Matrix<double, 7, 1> gradient = Eigen::Matrix<double, 7, 1>::Zero();
        
        for (int i = 0; i < 7; ++i) {
            double range = q_max_[i] - q_min_[i];
            double center = (q_max_[i] + q_min_[i]) / 2.0;
            double normalized_dist = std::abs(q[i] - center) / (range / 2.0);
            
            if (normalized_dist > 0.8) {
                double sign = (q[i] > center) ? 1.0 : -1.0;
                gradient[i] = sign * 10.0 * std::exp(10.0 * (normalized_dist - 0.8)) / (range / 2.0);
            }
        }
        return gradient;
    }

    // Enhanced IK solving with three objectives
    std::optional<Eigen::Matrix<double, 7, 1>> solve(
        const Eigen::Isometry3d& target_pose,
        const Eigen::Matrix<double, 7, 1>& q_init,
        const Eigen::Matrix<double, 7, 1>& q_current_target) {
        
        // Validate input
        if (!target_pose.matrix().allFinite() || !q_init.allFinite()) {
            std::cout << "IK solver: Invalid input data (NaN/Inf detected)" << std::endl;
            return std::nullopt;
        }
        
        Eigen::Matrix<double, 7, 1> q_current = q_init;
        
        for (int iter = 0; iter < ik_params_.max_iterations; ++iter) {
            // Forward kinematics
            auto fk_result = panda_kinematics::Kinematics::forward(q_current);
            Eigen::Isometry3d current_pose;
            Eigen::Matrix4d temp_matrix = Eigen::Map<const Eigen::Matrix4d>(fk_result.data());
            current_pose.matrix() = temp_matrix;
            
            // Calculate pose error
            Eigen::Matrix<double, 6, 1> pose_error;
            pose_error.head<3>() = target_pose.translation() - current_pose.translation();
            
            // Orientation error (axis-angle representation)
            Eigen::Matrix3d rotation_error = target_pose.rotation() * current_pose.rotation().transpose();
            Eigen::AngleAxisd angle_axis(rotation_error);
            if (std::abs(angle_axis.angle()) > 1e-8) {
                pose_error.tail<3>() = angle_axis.angle() * angle_axis.axis();
            } else {
                pose_error.tail<3>().setZero();
            }
            
            // Check convergence
            double pos_error_norm = pose_error.head<3>().norm();
            double rot_error_norm = pose_error.tail<3>().norm();
            
            if (pos_error_norm < ik_params_.position_tolerance &&
                rot_error_norm < ik_params_.orientation_tolerance) {
                return q_current;
            }
            
            // Calculate Jacobian
            auto jacobian = panda_kinematics::Kinematics::jacobian(q_current);
            
            if (!jacobian.allFinite()) {
                std::cout << "IK solver: Jacobian contains NaN/Inf at iteration " << iter << std::endl;
                return std::nullopt;
            }
            
            // Damped least squares for primary task
            Eigen::Matrix<double, 6, 6> damping = ik_params_.damping_factor * Eigen::Matrix<double, 6, 6>::Identity();
            Eigen::Matrix<double, 6, 6> JJT = jacobian * jacobian.transpose() + damping;
            
            // Check condition number
            Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(JJT);
            double condition_number = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
            if (condition_number > 1e12) {
                damping *= 10.0;
                JJT = jacobian * jacobian.transpose() + damping;
            }
            
            Eigen::Matrix<double, 7, 6> J_pinv = jacobian.transpose() * JJT.inverse();
            
            // Primary task: pose error minimization
            Eigen::Matrix<double, 7, 1> dq_primary = J_pinv * pose_error;
            
            // Secondary objectives
            // 1. Manipulability gradient (numerical)
            Eigen::Matrix<double, 7, 1> manip_gradient = Eigen::Matrix<double, 7, 1>::Zero();
            const double epsilon = 1e-6;
            
            for (int i = 0; i < 7; ++i) {
                Eigen::Matrix<double, 7, 1> q_plus = q_current;
                Eigen::Matrix<double, 7, 1> q_minus = q_current;
                q_plus[i] += epsilon;
                q_minus[i] -= epsilon;
                
                auto jac_plus = panda_kinematics::Kinematics::jacobian(q_plus);
                auto jac_minus = panda_kinematics::Kinematics::jacobian(q_minus);
                
                double manip_plus = calculateManipulability(jac_plus);
                double manip_minus = calculateManipulability(jac_minus);
                
                double gradient_val = (manip_plus - manip_minus) / (2.0 * epsilon);
                if (std::isfinite(gradient_val)) {
                    manip_gradient[i] = gradient_val;
                }
            }
            
            // 2. Joint limit avoidance gradient
            Eigen::Matrix<double, 7, 1> joint_limit_gradient = calculateJointLimitGradient(q_current);
            
            // 3. Configuration distance gradient
            Eigen::Matrix<double, 7, 1> config_dist_gradient = 
                calculateConfigDistanceGradient(q_current, q_current_target);
            
            // Null space projector
            Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
            Eigen::Matrix<double, 7, 7> null_space_proj = I - J_pinv * jacobian;
            
            // Combined secondary objective in null space
            Eigen::Matrix<double, 7, 1> dq_secondary = 
                ik_params_.manipulability_weight * manip_gradient -
                ik_params_.joint_limit_weight * joint_limit_gradient -
                ik_params_.config_distance_weight * config_dist_gradient;
            
            // Total motion
            Eigen::Matrix<double, 7, 1> dq_total = dq_primary + null_space_proj * dq_secondary;
            
            if (!dq_total.allFinite()) {
                std::cout << "IK solver: Motion contains NaN/Inf at iteration " << iter << std::endl;
                return std::nullopt;
            }
            
            // Limit step size
            double max_step = 0.1;
            if (dq_total.norm() > max_step) {
                dq_total = dq_total.normalized() * max_step;
            }
            
            // Update joint angles
            q_current += ik_params_.step_size * dq_total;
            
            // Clamp to joint limits
            for (int i = 0; i < 7; ++i) {
                q_current[i] = std::clamp(q_current[i], q_min_[i], q_max_[i]);
            }
        }
        
        std::cout << "IK failed to converge after " << ik_params_.max_iterations << " iterations" << std::endl;
        return std::nullopt;
    }

    // Set solver parameters
    void setParameters(double position_tol, double orientation_tol, 
                      double manip_weight, double joint_limit_weight, double config_dist_weight) {
        ik_params_.position_tolerance = position_tol;
        ik_params_.orientation_tolerance = orientation_tol;
        ik_params_.manipulability_weight = manip_weight;
        ik_params_.joint_limit_weight = joint_limit_weight;
        ik_params_.config_distance_weight = config_dist_weight;
    }
};

class HybridVRController
{
private:
    std::atomic<bool> running_{true};
    std::atomic<bool> should_stop_{false};
    VRCommand current_vr_command_;
    std::mutex command_mutex_;

    int server_socket_;
    const int PORT = 8888;

    // Enhanced IK solver
    EnhancedIKSolver ik_solver_;

    // Error recovery parameters (from Script 2)
    struct ErrorRecoveryParams
    {
        int max_recovery_attempts = 3;
        int recovery_delay_ms = 1000;
        bool enable_auto_recovery = true;
    } recovery_params_;

    // VR control parameters (simplified from Script 2)
    struct VRParams
    {
        double position_gain = 0.0010;           // Balanced gain
        double orientation_gain = 0.0008;        // Balanced gain
        double vr_smoothing = 0.1;               // Smoothing of incoming VR data
        double position_deadzone = 0.001;        // 1mm deadzone
        double orientation_deadzone = 0.03;      // ~1.7 degrees deadzone
        double max_interp_position_step = 0.3;   // 0.3m/s max interpolation speed
        double max_interp_orientation_step = 0.6; // 0.6rad/s max interpolation speed
        double max_position_offset = 0.6;        // 60cm workspace limit
        double max_joint_change_per_cycle = 0.001; // Smooth joint transitions
    } vr_params_;

    // Target poses
    Eigen::Isometry3d vr_target_pose_;
    Eigen::Isometry3d interpolated_target_pose_;

    // VR filtering state
    Eigen::Vector3d filtered_vr_position_{0, 0, 0};
    Eigen::Quaterniond filtered_vr_orientation_{1, 0, 0, 0};

    // Initial poses and joint configuration
    Eigen::Isometry3d initial_robot_pose_;
    Eigen::Vector3d initial_vr_position_{0, 0, 0};
    Eigen::Quaterniond initial_vr_orientation_{1, 0, 0, 0};
    Eigen::Matrix<double, 7, 1> current_joint_target_;
    bool vr_initialized_ = false;

    // Performance monitoring
    struct PerformanceStats {
        std::atomic<int> ik_failures_{0};
        std::atomic<int> ik_successes_{0};
        std::atomic<double> avg_solve_time_{0.0};
    } stats_;

public:
    HybridVRController()
    {
        setupNetworking();
        
        // Initialize enhanced IK solver parameters
        ik_solver_.setParameters(1e-4, 1e-3, 0.1, 0.05, 0.2);
    }

    ~HybridVRController()
    {
        running_ = false;
        close(server_socket_);
    }

    void stop()
    {
        should_stop_ = true;
        running_ = false;
    }

    void setupNetworking()
    {
        server_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (server_socket_ < 0)
        {
            throw std::runtime_error("Failed to create socket");
        }

        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(PORT);

        if (bind(server_socket_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
        {
            throw std::runtime_error("Failed to bind socket");
        }

        int flags = fcntl(server_socket_, F_GETFL, 0);
        fcntl(server_socket_, F_SETFL, flags | O_NONBLOCK);

        std::cout << "UDP server listening on port " << PORT << " for VR pose data" << std::endl;
    }

    void networkThread()
    {
        char buffer[1024];
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);

        while (running_)
        {
            ssize_t bytes_received = recvfrom(server_socket_, buffer, sizeof(buffer), 0,
                                              (struct sockaddr *)&client_addr, &client_len);

            if (bytes_received > 0)
            {
                buffer[bytes_received] = '\0';

                VRCommand cmd;
                int parsed_count = sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf",
                                          &cmd.pos_x, &cmd.pos_y, &cmd.pos_z,
                                          &cmd.quat_x, &cmd.quat_y, &cmd.quat_z, &cmd.quat_w);

                if (parsed_count == 7)
                {
                    cmd.has_valid_data = true;

                    std::lock_guard<std::mutex> lock(command_mutex_);
                    current_vr_command_ = cmd;

                    if (!vr_initialized_)
                    {
                        initial_vr_position_ = Eigen::Vector3d(cmd.pos_x, cmd.pos_y, cmd.pos_z);
                        initial_vr_orientation_ = Eigen::Quaterniond(cmd.quat_w, cmd.quat_x, cmd.quat_y, cmd.quat_z).normalized();

                        filtered_vr_position_ = initial_vr_position_;
                        filtered_vr_orientation_ = initial_vr_orientation_;

                        // Reset VR target to current robot pose when VR initializes
                        vr_target_pose_ = initial_robot_pose_;
                        interpolated_target_pose_ = initial_robot_pose_;

                        vr_initialized_ = true;
                        std::cout << "VR reference pose initialized!" << std::endl;
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

private:
    // Clean VR target updates (from Script 2)
    void updateVRTargets(const VRCommand &cmd)
    {
        if (!cmd.has_valid_data || !vr_initialized_)
        {
            return;
        }

        // Current VR pose
        Eigen::Vector3d vr_pos(cmd.pos_x, cmd.pos_y, cmd.pos_z);
        Eigen::Quaterniond vr_quat(cmd.quat_w, cmd.quat_x, cmd.quat_y, cmd.quat_z);
        vr_quat.normalize();

        // Smooth incoming VR data
        double alpha = 1.0 - vr_params_.vr_smoothing;
        filtered_vr_position_ = vr_params_.vr_smoothing * filtered_vr_position_ + alpha * vr_pos;
        filtered_vr_orientation_ = filtered_vr_orientation_.slerp(alpha, vr_quat);

        // Calculate deltas from initial VR pose
        Eigen::Vector3d vr_pos_delta = filtered_vr_position_ - initial_vr_position_;
        Eigen::Quaterniond vr_quat_delta = filtered_vr_orientation_ * initial_vr_orientation_.inverse();

        // Apply deadzones
        if (vr_pos_delta.norm() < vr_params_.position_deadzone)
        {
            vr_pos_delta.setZero();
        }
        double rotation_angle = 2.0 * acos(std::abs(vr_quat_delta.w()));
        if (rotation_angle < vr_params_.orientation_deadzone)
        {
            vr_quat_delta.setIdentity();
        }

        // Apply workspace limits
        if (vr_pos_delta.norm() > vr_params_.max_position_offset)
        {
            vr_pos_delta = vr_pos_delta.normalized() * vr_params_.max_position_offset;
        }

        // Apply gains to VR deltas
        vr_pos_delta *= vr_params_.position_gain;
        
        // Scale rotation by orientation gain  
        Eigen::AngleAxisd scaled_rotation(vr_quat_delta);
        scaled_rotation.angle() *= vr_params_.orientation_gain;
        
        // Safety limit on rotation
        if (std::abs(scaled_rotation.angle()) > 0.1) {
            scaled_rotation.angle() = std::copysign(0.1, scaled_rotation.angle());
        }
        
        Eigen::Quaterniond scaled_quat_delta(scaled_rotation);
        
        // Update target pose
        vr_target_pose_.translation() = initial_robot_pose_.translation() + vr_pos_delta;
        vr_target_pose_.linear() = (scaled_quat_delta * Eigen::Quaterniond(initial_robot_pose_.rotation())).toRotationMatrix();
    }

    // Error recovery with manual confirmation (from Script 2)
    bool attemptErrorRecovery(franka::Robot &robot, const franka::Exception &e, int attempt)
    {
        if (!recovery_params_.enable_auto_recovery)
        {
            return false;
        }

        std::cout << "\n========================================" << std::endl;
        std::cout << "ROBOT ERROR DETECTED!" << std::endl;
        std::cout << "Error: " << e.what() << std::endl;
        std::cout << "Recovery attempt " << attempt << "/" << recovery_params_.max_recovery_attempts << std::endl;
        std::cout << "========================================" << std::endl;

        std::cout << "\nPress Enter to attempt recovery (or Ctrl+C to exit): ";
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin.get();

        try
        {
            std::cout << "Attempting automatic error recovery..." << std::endl;
            robot.automaticErrorRecovery();
            std::cout << "✓ Automatic error recovery successful!" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "Robot ready to continue operation." << std::endl;
            return true;
        }
        catch (const franka::Exception &recovery_error)
        {
            std::cout << "✗ Error recovery failed: " << recovery_error.what() << std::endl;
            return false;
        }
    }

    void printPerformanceStats()
    {
        int total_attempts = stats_.ik_successes_ + stats_.ik_failures_;
        if (total_attempts > 0) {
            double success_rate = 100.0 * stats_.ik_successes_ / total_attempts;
            std::cout << "IK Performance - Success Rate: " << success_rate << "% (" 
                      << stats_.ik_successes_ << "/" << total_attempts << ")" << std::endl;
            std::cout << "Average solve time: " << stats_.avg_solve_time_ << "ms" << std::endl;
        }
    }

public:
    void run(const std::string &robot_ip)
    {
        bool control_active = true;
        int recovery_attempts = 0;

        while (control_active && !should_stop_)
        {
            try
            {
                franka::Robot robot(robot_ip);
                setDefaultBehavior(robot);

                if (recovery_attempts == 0)
                {
                    // Move to initial joint configuration
                    std::array<double, 7> q_goal = {{60.0 * M_PI / 180.0,
                                                     -55.0 * M_PI / 180.0,
                                                     -70.0 * M_PI / 180.0,
                                                     -100.0 * M_PI / 180.0,
                                                     -30.0 * M_PI / 180.0,
                                                     160.0 * M_PI / 180.0,
                                                     30.0 * M_PI / 180.0}};
                    MotionGenerator motion_generator(0.5, q_goal);
                    std::cout << "WARNING: This example will move the robot! "
                              << "Please make sure to have the user stop button at hand!" << std::endl
                              << "Press Enter to continue..." << std::endl;
                    std::cin.ignore();
                    robot.control(motion_generator);
                    std::cout << "Finished moving to initial joint configuration." << std::endl;

                    // Set robot parameters for joint control
                    robot.setCollisionBehavior(
                        {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}}, {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}},
                        {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}}, {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}},
                        {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}},
                        {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}});

                    // Set joint impedance for smooth motion
                    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
                }

                // Initialize poses and joint targets using consistent FK
                franka::RobotState state = robot.readOnce();
                
                Eigen::Matrix<double, 7, 1> initial_q = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.q.data());
                auto initial_fk = panda_kinematics::Kinematics::forward(initial_q);
                Eigen::Matrix4d initial_fk_matrix = Eigen::Map<const Eigen::Matrix4d>(initial_fk.data());
                initial_robot_pose_.matrix() = initial_fk_matrix;
                
                // Initialize targets
                vr_target_pose_ = initial_robot_pose_;
                interpolated_target_pose_ = initial_robot_pose_;
                current_joint_target_ = initial_q;
                
                std::cout << "Robot initialized at position: [" << initial_robot_pose_.translation().transpose() << "]" << std::endl;

                std::thread network_thread(&HybridVRController::networkThread, this);

                std::cout << "Waiting for VR data..." << std::endl;
                while (!vr_initialized_ && running_ && !should_stop_)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                if (vr_initialized_ && !should_stop_)
                {
                    std::cout << "VR initialized! Starting hybrid joint-level control." << std::endl;
                    this->runHybridVRControl(robot);
                }

                running_ = false;
                if (network_thread.joinable())
                    network_thread.join();

                printPerformanceStats();
                control_active = false;
            }
            catch (const franka::Exception &e)
            {
                std::cerr << "Franka exception: " << e.what() << std::endl;
                running_ = false;

                recovery_attempts++;
                if (recovery_attempts <= recovery_params_.max_recovery_attempts)
                {
                    try
                    {
                        franka::Robot recovery_robot(robot_ip);
                        if (attemptErrorRecovery(recovery_robot, e, recovery_attempts))
                        {
                            running_ = true;
                            vr_initialized_ = false;
                            continue;
                        }
                    }
                    catch (const franka::Exception &recovery_error)
                    {
                        std::cerr << "Failed to create robot instance for recovery: " << recovery_error.what() << std::endl;
                    }
                }
                control_active = false;
            }
        }
    }

private:
    void runHybridVRControl(franka::Robot &robot)
    {
        auto joint_control_callback = [this](
                                         const franka::RobotState &robot_state,
                                         franka::Duration period) -> franka::JointPositions
        {
            if (should_stop_)
            {
                return franka::MotionFinished(franka::JointPositions(robot_state.q_d));
            }

            // Update VR targets
            VRCommand cmd;
            {
                std::lock_guard<std::mutex> lock(command_mutex_);
                cmd = current_vr_command_;
            }
            updateVRTargets(cmd);

            // Interpolate target pose towards VR target
            const double max_step_distance = vr_params_.max_interp_position_step * period.toSec();
            const double max_step_angle = vr_params_.max_interp_orientation_step * period.toSec();

            // Position interpolation
            Eigen::Vector3d pos_error = vr_target_pose_.translation() - interpolated_target_pose_.translation();
            double distance = pos_error.norm();
            if (distance > 1e-6)
            {
                double step_distance = std::min(max_step_distance, distance);
                interpolated_target_pose_.translation() += (pos_error / distance) * step_distance;
            }

            // Orientation interpolation
            Eigen::Quaterniond current_quat(interpolated_target_pose_.rotation());
            Eigen::Quaterniond target_quat(vr_target_pose_.rotation());
            double angle_error = current_quat.angularDistance(target_quat);
            if (angle_error > 1e-6)
            {
                double step_angle = std::min(max_step_angle, angle_error);
                double slerp_fraction = step_angle / angle_error;
                Eigen::Quaterniond interpolated_quat = current_quat.slerp(slerp_fraction, target_quat);
                interpolated_target_pose_.linear() = interpolated_quat.toRotationMatrix();
            }

            // Solve enhanced IK with multi-objective optimization
            auto start_time = std::chrono::high_resolution_clock::now();
            
            Eigen::Matrix<double, 7, 1> current_q = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
            
            // Quick proximity check to skip IK if already very close
            auto current_fk = panda_kinematics::Kinematics::forward(current_q);
            Eigen::Matrix4d current_pose_matrix = Eigen::Map<const Eigen::Matrix4d>(current_fk.data());
            Eigen::Isometry3d current_pose;
            current_pose.matrix() = current_pose_matrix;
            
            double pos_diff = (interpolated_target_pose_.translation() - current_pose.translation()).norm();
            Eigen::Matrix3d rot_diff = interpolated_target_pose_.rotation() * current_pose.rotation().transpose();
            Eigen::AngleAxisd angle_axis(rot_diff);
            double rot_diff_angle = std::abs(angle_axis.angle());
            
            std::optional<Eigen::Matrix<double, 7, 1>> ik_result;
            
            // Skip IK if already very close to save computation
            if (pos_diff < 0.0005 && rot_diff_angle < 0.005) {
                ik_result = current_q;
                stats_.ik_successes_++;
            } else {
                // Solve IK with enhanced multi-objective optimization
                // Pass current joint target for configuration distance minimization
                ik_result = ik_solver_.solve(interpolated_target_pose_, current_q, current_joint_target_);
            }
            
            auto end_time = std::chrono::high_resolution_clock::now();
            auto solve_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0;
            
            // Update performance statistics
            static double avg_time = 0.0;
            static int sample_count = 0;
            avg_time = (avg_time * sample_count + solve_time) / (sample_count + 1);
            sample_count++;
            if (sample_count % 1000 == 0) {
                stats_.avg_solve_time_ = avg_time;
            }

            if (ik_result.has_value())
            {
                Eigen::Matrix<double, 7, 1> new_target = ik_result.value();
                if (new_target.allFinite()) {
                    // Smooth joint transition to prevent velocity discontinuities
                    Eigen::Matrix<double, 7, 1> joint_diff = new_target - current_joint_target_;
                    double change_magnitude = joint_diff.norm();
                    
                    if (change_magnitude > vr_params_.max_joint_change_per_cycle) {
                        // Limit the change to prevent large jumps
                        joint_diff = joint_diff.normalized() * vr_params_.max_joint_change_per_cycle;
                        current_joint_target_ += joint_diff;
                    } else {
                        current_joint_target_ = new_target;
                    }
                    stats_.ik_successes_++;
                } else {
                    std::cout << "IK result rejected: contains NaN/Inf" << std::endl;
                    stats_.ik_failures_++;
                }
            }
            else
            {
                // IK failed - maintain current position
                stats_.ik_failures_++;
                // current_joint_target_ remains unchanged
            }

            // Convert to array format for libfranka
            std::array<double, 7> joint_positions;
            for (int i = 0; i < 7; ++i)
            {
                joint_positions[i] = current_joint_target_[i];
                
                // Safety check for finite values
                if (!std::isfinite(joint_positions[i])) {
                    std::cout << "ERROR: Non-finite joint position[" << i << "] = " << joint_positions[i] << std::endl;
                    return franka::MotionFinished(franka::JointPositions(robot_state.q_d));
                }
            }

            if (!running_)
            {
                return franka::MotionFinished(franka::JointPositions(joint_positions));
            }

            return franka::JointPositions(joint_positions);
        };

        try {
            robot.control(joint_control_callback);
        } catch (const franka::Exception& e) {
            std::cout << "Franka exception in control loop: " << e.what() << std::endl;
            throw;
        } catch (const std::exception& e) {
            std::cout << "Standard exception in control loop: " << e.what() << std::endl;
            throw;
        }
    }
};

// Global pointer for signal handler
HybridVRController* g_controller = nullptr;

void signalHandler(int signum)
{
    std::cout << "\nReceived signal " << signum << ". Shutting down gracefully..." << std::endl;
    if (g_controller)
    {
        g_controller->stop();
    }
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    try
    {
        HybridVRController controller;
        g_controller = &controller;

        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);

        std::cout << "========================================" << std::endl;
        std::cout << "HYBRID VR TELEOPERATION CONTROLLER" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Features:" << std::endl;
        std::cout << "- Enhanced IK with multi-objective optimization" << std::endl;
        std::cout << "- Manipulability maximization" << std::endl;
        std::cout << "- Joint limit avoidance" << std::endl;
        std::cout << "- Configuration distance minimization" << std::endl;
        std::cout << "- Robust error recovery" << std::endl;
        std::cout << "- Performance monitoring" << std::endl;
        std::cout << "========================================" << std::endl;

        controller.run(argv[1]);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}