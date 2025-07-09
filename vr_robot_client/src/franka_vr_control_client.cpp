// VR-Based Cartesian Teleoperation - Robust Trajectory Generation with Jerk Limiting
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

struct VRCommand
{
    double pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;
    double quat_x = 0.0, quat_y = 0.0, quat_z = 0.0, quat_w = 1.0;
    bool has_valid_data = false;
};

// Enhanced trajectory state with acceleration tracking
struct TrajectoryState
{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration; // Added for jerk limiting
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d angular_acceleration; // Added for angular jerk limiting
    double time;

    TrajectoryState()
    {
        position.setZero();
        velocity.setZero();
        acceleration.setZero();
        orientation.setIdentity();
        angular_velocity.setZero();
        angular_acceleration.setZero();
        time = 0.0;
    }
};

class RobustVRController
{
private:
    std::atomic<bool> running_{true};
    VRCommand current_vr_command_;
    std::mutex command_mutex_;

    int server_socket_;
    const int PORT = 8888;

    // Enhanced trajectory parameters with jerk limiting
    struct TrajParams
    {
        // Motion limits - conservative values for stability
        double max_velocity = 0.8;             // 5cm/s
        double max_acceleration = 0.04;        // 1.5cm/s²
        double max_jerk = 0.01;                 // 5cm/s³ - NEW: jerk limit
        double max_angular_velocity = 0.1;      // 0.1 rad/s (~6 deg/s)
        double max_angular_acceleration = 0.04; // 0.02 rad/s²
        double max_angular_jerk = 0.02;          // 0.1 rad/s³ - NEW: angular jerk limit

        // VR mapping parameters - reduced for stability
        double position_responsiveness = 0.3;    // Reduced from 0.25
        double orientation_responsiveness = 0.2; // Reduced from 0.25
        double vr_smoothing = 0.8;                // Increased from 0.8

        // Target smoothing - NEW: smooth target changes
        double target_position_smoothing = 0.8;
        double target_orientation_smoothing = 0.8;

        // Deadzones
        double position_deadzone = 0.002;   // 2mm
        double orientation_deadzone = 0.03; // ~1.7 degrees

        // Workspace limits
        double max_position_offset = 0.8;    // 15cm from initial position
        double max_orientation_offset = 0.8; // ~14 degrees from initial orientation
    } params_;

    // State
    TrajectoryState current_state_;
    Eigen::Vector3d target_position_;
    Eigen::Quaterniond target_orientation_;

    // NEW: Smoothed targets to prevent sudden jumps
    Eigen::Vector3d smoothed_target_position_;
    Eigen::Quaterniond smoothed_target_orientation_;

    // VR filtering
    Eigen::Vector3d filtered_vr_position_{0, 0, 0};
    Eigen::Quaterniond filtered_vr_orientation_{1, 0, 0, 0};

    // Initial poses
    Eigen::Affine3d initial_robot_pose_;
    Eigen::Vector3d initial_vr_position_{0, 0, 0};
    Eigen::Quaterniond initial_vr_orientation_{1, 0, 0, 0};
    bool vr_initialized_ = false;

public:
    RobustVRController()
    {
        setupNetworking();
    }

    ~RobustVRController()
    {
        running_ = false;
        close(server_socket_);
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

        // Heavy smoothing on VR input
        double alpha = 1.0 - params_.vr_smoothing;
        filtered_vr_position_ = params_.vr_smoothing * filtered_vr_position_ + alpha * vr_pos;
        filtered_vr_orientation_ = filtered_vr_orientation_.slerp(alpha, vr_quat);

        // Calculate deltas from initial VR pose
        Eigen::Vector3d vr_pos_delta = filtered_vr_position_ - initial_vr_position_;
        Eigen::Quaterniond vr_quat_delta = filtered_vr_orientation_ * initial_vr_orientation_.inverse();

        // Apply deadzones
        if (vr_pos_delta.norm() < params_.position_deadzone)
        {
            vr_pos_delta.setZero();
        }

        double rotation_angle = 2.0 * std::acos(std::abs(vr_quat_delta.w()));
        if (rotation_angle < params_.orientation_deadzone)
        {
            vr_quat_delta.setIdentity();
        }

        // Apply workspace limits
        if (vr_pos_delta.norm() > params_.max_position_offset)
        {
            vr_pos_delta = vr_pos_delta.normalized() * params_.max_position_offset;
        }

        if (rotation_angle > params_.max_orientation_offset)
        {
            Eigen::AngleAxisd axis_angle(vr_quat_delta);
            double scale = params_.max_orientation_offset / rotation_angle;
            vr_quat_delta = Eigen::Quaterniond(Eigen::AngleAxisd(axis_angle.angle() * scale, axis_angle.axis()));
        }

        // Calculate raw targets
        Eigen::Vector3d raw_target_position = initial_robot_pose_.translation() + vr_pos_delta;
        Eigen::Quaterniond raw_target_orientation = vr_quat_delta * Eigen::Quaterniond(initial_robot_pose_.rotation());
        raw_target_orientation.normalize();

        // NEW: Smooth target changes to prevent sudden jumps
        double pos_smooth = params_.target_position_smoothing;
        double orient_smooth = params_.target_orientation_smoothing;

        smoothed_target_position_ = pos_smooth * smoothed_target_position_ +
                                    (1.0 - pos_smooth) * raw_target_position;

        smoothed_target_orientation_ = smoothed_target_orientation_.slerp(1.0 - orient_smooth, raw_target_orientation);
        smoothed_target_orientation_.normalize();

        // Use smoothed targets
        target_position_ = smoothed_target_position_;
        target_orientation_ = smoothed_target_orientation_;
    }

    // Enhanced trajectory generation with jerk limiting
    TrajectoryState generateNextState(double dt)
    {
        TrajectoryState next_state = current_state_;
        next_state.time += dt;

        // JERK-LIMITED POSITION CONTROL
        Eigen::Vector3d position_error = target_position_ - current_state_.position;
        Eigen::Vector3d desired_velocity = position_error * params_.position_responsiveness;

        // Limit desired velocity
        if (desired_velocity.norm() > params_.max_velocity)
        {
            desired_velocity = desired_velocity.normalized() * params_.max_velocity;
        }

        // Calculate desired acceleration
        Eigen::Vector3d velocity_error = desired_velocity - current_state_.velocity;
        Eigen::Vector3d desired_acceleration = velocity_error * params_.position_responsiveness;

        // Limit desired acceleration
        if (desired_acceleration.norm() > params_.max_acceleration)
        {
            desired_acceleration = desired_acceleration.normalized() * params_.max_acceleration;
        }

        // NEW: Jerk-limited acceleration change
        Eigen::Vector3d acceleration_error = desired_acceleration - current_state_.acceleration;
        Eigen::Vector3d jerk = acceleration_error / dt;

        // Apply jerk limits
        if (jerk.norm() > params_.max_jerk)
        {
            jerk = jerk.normalized() * params_.max_jerk;
        }

        // Update acceleration, velocity, and position with jerk limiting
        next_state.acceleration = current_state_.acceleration + jerk * dt;
        next_state.velocity = current_state_.velocity + next_state.acceleration * dt;
        next_state.position = current_state_.position + next_state.velocity * dt;

        // IMPROVED ORIENTATION CONTROL with angular jerk limiting
        double slerp_factor = params_.orientation_responsiveness * dt;
        slerp_factor = std::min(slerp_factor, 0.1); // Max 10% per step

        // Use SLERP to smoothly approach target orientation
        next_state.orientation = current_state_.orientation.slerp(slerp_factor, target_orientation_);
        next_state.orientation.normalize();

        // Calculate angular velocity from orientation change
        Eigen::Quaterniond orientation_diff = next_state.orientation * current_state_.orientation.inverse();
        orientation_diff.normalize();

        // Ensure shortest path
        if (orientation_diff.w() < 0)
        {
            orientation_diff.coeffs() *= -1;
        }

        // Convert orientation change to desired angular velocity
        Eigen::Vector3d desired_angular_velocity;
        if (dt > 1e-8)
        {
            double angle = 2.0 * std::acos(std::abs(orientation_diff.w()));
            if (angle > 1e-6)
            {
                double sin_half_angle = std::sqrt(1.0 - orientation_diff.w() * orientation_diff.w());
                if (sin_half_angle > 1e-6)
                {
                    Eigen::Vector3d axis = Eigen::Vector3d(orientation_diff.x(), orientation_diff.y(), orientation_diff.z()) / sin_half_angle;
                    desired_angular_velocity = axis * angle / dt;
                }
                else
                {
                    desired_angular_velocity.setZero();
                }
            }
            else
            {
                desired_angular_velocity.setZero();
            }
        }
        else
        {
            desired_angular_velocity.setZero();
        }

        // Limit desired angular velocity
        if (desired_angular_velocity.norm() > params_.max_angular_velocity)
        {
            desired_angular_velocity = desired_angular_velocity.normalized() * params_.max_angular_velocity;
        }

        // NEW: Angular acceleration limiting (angular jerk limiting)
        Eigen::Vector3d angular_velocity_error = desired_angular_velocity - current_state_.angular_velocity;
        Eigen::Vector3d desired_angular_acceleration = angular_velocity_error / dt;

        // Limit angular acceleration change (angular jerk)
        Eigen::Vector3d angular_acceleration_error = desired_angular_acceleration - current_state_.angular_acceleration;
        Eigen::Vector3d angular_jerk = angular_acceleration_error / dt;

        if (angular_jerk.norm() > params_.max_angular_jerk)
        {
            angular_jerk = angular_jerk.normalized() * params_.max_angular_jerk;
        }

        // Update angular acceleration and velocity
        next_state.angular_acceleration = current_state_.angular_acceleration + angular_jerk * dt;

        // Apply angular acceleration limits
        if (next_state.angular_acceleration.norm() > params_.max_angular_acceleration)
        {
            next_state.angular_acceleration = next_state.angular_acceleration.normalized() * params_.max_angular_acceleration;
        }

        next_state.angular_velocity = current_state_.angular_velocity + next_state.angular_acceleration * dt;

        // Apply final angular velocity limits
        if (next_state.angular_velocity.norm() > params_.max_angular_velocity)
        {
            next_state.angular_velocity = next_state.angular_velocity.normalized() * params_.max_angular_velocity;
        }

        return next_state;
    }

    franka::CartesianPose stateToCartesianPose(const TrajectoryState &state)
    {
        Eigen::Affine3d pose;
        pose.translation() = state.position;
        pose.linear() = state.orientation.toRotationMatrix();

        std::array<double, 16> pose_array;
        Eigen::Map<Eigen::Matrix4d>(pose_array.data()) = pose.matrix();
        return franka::CartesianPose(pose_array);
    }

    void initializeSmoothedTargets()
    {
        smoothed_target_position_ = current_state_.position;
        smoothed_target_orientation_ = current_state_.orientation;
    }

public:
    void run(const std::string &robot_ip)
    {
        try
        {
            std::cout << "Connecting to robot at " << robot_ip << std::endl;
            franka::Robot robot(robot_ip);
            setDefaultBehavior(robot);

            // Move to initial configuration
            std::array<double, 7> q_goal = {{60.0 * M_PI / 180.0,
                                             -55.0 * M_PI / 180.0,
                                             -70.0 * M_PI / 180.0,
                                             -100.0 * M_PI / 180.0,
                                             -30.0 * M_PI / 180.0,
                                             160.0 * M_PI / 180.0,
                                             30.0 * M_PI / 180.0}};
            MotionGenerator motion_generator(0.5, q_goal);
            std::cout << "WARNING: This example will move the robot!" << std::endl
                      << "Press Enter to continue..." << std::endl;
            std::cin.ignore();
            robot.control(motion_generator);

            // Conservative collision behavior
            robot.setCollisionBehavior(
                {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}}, {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}},
                {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}}, {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}},
                {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}},
                {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}});

            // Moderate impedance for smooth motion
            robot.setCartesianImpedance({{1000, 1000, 1000, 100, 100, 100}});

            // Initialize from current pose
            franka::RobotState state = robot.readOnce();
            initial_robot_pose_ = Eigen::Affine3d(Eigen::Matrix4d::Map(state.O_T_EE_d.data()));

            current_state_.position = initial_robot_pose_.translation();
            current_state_.orientation = Eigen::Quaterniond(initial_robot_pose_.rotation());
            current_state_.velocity.setZero();
            current_state_.acceleration.setZero();
            current_state_.angular_velocity.setZero();
            current_state_.angular_acceleration.setZero();

            target_position_ = current_state_.position;
            target_orientation_ = current_state_.orientation;

            // Initialize smoothed targets
            initializeSmoothedTargets();

            std::thread network_thread(&RobustVRController::networkThread, this);

            std::cout << "Waiting for VR data..." << std::endl;
            while (!vr_initialized_ && running_)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (vr_initialized_)
            {
                std::cout << "VR initialized! Starting jerk-limited trajectory control..." << std::endl;
                this->runTrajectoryControl(robot);
            }

            running_ = false;
            network_thread.join();
        }
        catch (const franka::Exception &e)
        {
            std::cout << "Franka exception: " << e.what() << std::endl;
            running_ = false;
        }
    }

private:
    void runTrajectoryControl(franka::Robot &robot)
    {
        std::cout << "Starting jerk-limited VR trajectory control..." << std::endl;

        try
        {
            robot.automaticErrorRecovery();
        }
        catch (const franka::Exception &e)
        {
            std::cout << "Error recovery: " << e.what() << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));

        int iteration_count = 0;

        auto trajectory_generator = [this, iteration_count = 0](const franka::RobotState &robot_state, franka::Duration period) mutable -> franka::CartesianPose
        {
            iteration_count++;
            double dt = period.toSec();

            // Get VR command
            VRCommand cmd;
            {
                std::lock_guard<std::mutex> lock(command_mutex_);
                cmd = current_vr_command_;
            }

            // Update targets and generate trajectory
            if (iteration_count > 10 && dt > 0.0 && dt < 0.01 && vr_initialized_)
            {
                updateVRTargets(cmd);
                current_state_ = generateNextState(dt);

                // Enhanced debug output
                if (iteration_count % 100 == 0)
                {
                    Eigen::Vector3d pos_error = target_position_ - current_state_.position;
                    Eigen::Quaterniond orient_error = target_orientation_ * current_state_.orientation.inverse();
                    double orient_angle = 2.0 * std::acos(std::abs(orient_error.w()));

                    std::cout << "Jerk-Limited VR Control:" << std::endl
                              << "  Pos error: " << pos_error.norm() << " m" << std::endl
                              << "  Orient error: " << (orient_angle * 180.0 / M_PI) << " deg" << std::endl
                              << "  Velocity: " << current_state_.velocity.norm() << " m/s" << std::endl
                              << "  Acceleration: " << current_state_.acceleration.norm() << " m/s²" << std::endl
                              << "  Angular vel: " << current_state_.angular_velocity.norm() << " rad/s" << std::endl
                              << "  Angular accel: " << current_state_.angular_acceleration.norm() << " rad/s²" << std::endl;
                }
            }

            return stateToCartesianPose(current_state_);
        };

        try
        {
            robot.control(trajectory_generator);
            std::cout << "Jerk-limited VR control finished normally." << std::endl;
        }
        catch (const franka::ControlException &e)
        {
            std::cout << "Jerk-limited VR control exception: " << e.what() << std::endl;
            std::cout << "Final state: pos=[" << current_state_.position.x() << ", "
                      << current_state_.position.y() << ", " << current_state_.position.z() << "]"
                      << " vel=" << current_state_.velocity.norm()
                      << " accel=" << current_state_.acceleration.norm() << std::endl;
        }
    }
};

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    try
    {
        RobustVRController controller;
        controller.run(argv[1]);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}