// VR-Based Cartesian Teleoperation - Final Version
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

#include <franka/exception.h>
#include <franka/robot.h>
#include <Eigen/Dense>

#include "examples_common.h"

struct VRCommand
{
    double pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;
    double quat_x = 0.0, quat_y = 0.0, quat_z = 0.0, quat_w = 1.0;
    bool has_valid_data = false;
};

class SimplifiedVRController
{
private:
    std::atomic<bool> running_{true};
    VRCommand current_vr_command_;
    std::mutex command_mutex_;

    int server_socket_;
    const int PORT = 8888;

    // Simplified parameters for VR mapping
    struct VRParams
    {
        double position_gain = 0.0015;    // control gain for position contorl
        double orientation_gain = 0.0015; // control gain for orientation control
        double vr_smoothing = 0.1;        // Smoothing of incoming VR data

        // Deadzones to prevent drift from small sensor noise
        double position_deadzone = 0.001;   // 1mm
        double orientation_deadzone = 0.03; // ~1.7 degrees

        // Interpolation max step size
        double max_interp_position_step = 0.2; // 0.2m/s
        double max_interp_orientation_step = 0.5; // 0.5rad/s

        // Workspace limits to keep the robot in a safe area
        double max_position_offset = 0.6;   // 60cm from initial position
    } params_;

    // Rename target_... to vr_target_... to clarify it's the goal from VR.
    Eigen::Vector3d vr_target_position_;
    Eigen::Quaterniond vr_target_orientation_;

    // This is the smooth target that updates at 1kHz.
    Eigen::Vector3d interpolated_target_position_;
    Eigen::Quaterniond interpolated_target_orientation_;

    // VR filtering state
    Eigen::Vector3d filtered_vr_position_{0, 0, 0};
    Eigen::Quaterniond filtered_vr_orientation_{1, 0, 0, 0};

    // Initial poses used as a reference frame
    Eigen::Affine3d initial_robot_pose_;
    Eigen::Vector3d initial_vr_position_{0, 0, 0};
    Eigen::Quaterniond initial_vr_orientation_{1, 0, 0, 0};
    bool vr_initialized_ = false;

public:
    SimplifiedVRController()
    {
        setupNetworking();
    }

    ~SimplifiedVRController()
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
                        initial_vr_orientation_ = Eigen::Quaterniond(cmd.quat_w, cmd.quat_x, cmd.quat_y, cmd.quat_z).normalized();

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
    // This function's only job is to calculate the desired target pose from VR data.
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

        // Smooth incoming VR data to reduce jitter
        double alpha = 1.0 - params_.vr_smoothing;
        filtered_vr_position_ = params_.vr_smoothing * filtered_vr_position_ + alpha * vr_pos;
        filtered_vr_orientation_ = filtered_vr_orientation_.slerp(alpha, vr_quat);

        // Calculate deltas from the initial VR pose
        Eigen::Vector3d vr_pos_delta = filtered_vr_position_ - initial_vr_position_;
        Eigen::Quaterniond vr_quat_delta = filtered_vr_orientation_ * initial_vr_orientation_.inverse();

        // Apply deadzones to prevent drift
        if (vr_pos_delta.norm() < params_.position_deadzone)
        {
            vr_pos_delta.setZero();
        }
        double rotation_angle = 2.0 * acos(std::abs(vr_quat_delta.w()));
        if (rotation_angle < params_.orientation_deadzone)
        {
            vr_quat_delta.setIdentity();
        }

        // Apply workspace limits
        if (vr_pos_delta.norm() > params_.max_position_offset)
        {
            vr_pos_delta = vr_pos_delta.normalized() * params_.max_position_offset;
        }

        // The final calculation just updates the vr_target_
        vr_target_position_ = initial_robot_pose_.translation() + vr_pos_delta;
        vr_target_orientation_ = vr_quat_delta * Eigen::Quaterniond(initial_robot_pose_.rotation());
        vr_target_orientation_.normalize();
    }

    std::array<double, 16> createPoseArray(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation)
    {
        Eigen::Affine3d pose;
        pose.translation() = position;
        pose.linear() = orientation.toRotationMatrix();

        std::array<double, 16> pose_array;
        Eigen::Map<Eigen::Matrix4d>(pose_array.data()) = pose.matrix();
        return pose_array;
    }

public:
    void run(const std::string &robot_ip)
    {
        try
        {
            franka::Robot robot(robot_ip);
            setDefaultBehavior(robot);

            // Move to a suitable starting joint configuration
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

            // Collision behavior
            robot.setCollisionBehavior(
                {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}}, {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}},
                {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}}, {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}},
                {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}},
                {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}});

            // Moderate impedance for smooth motion
            robot.setCartesianImpedance({{1000, 1000, 1000, 100, 100, 100}});

            // Initialize poses from the robot's current state
            franka::RobotState state = robot.readOnce();
            initial_robot_pose_ = Eigen::Affine3d(Eigen::Matrix4d::Map(state.O_T_EE.data()));

            // Initialize all targets to the robot's starting pose
            vr_target_position_ = initial_robot_pose_.translation();
            interpolated_target_position_ = initial_robot_pose_.translation();

            vr_target_orientation_ = Eigen::Quaterniond(initial_robot_pose_.rotation());
            interpolated_target_orientation_ = Eigen::Quaterniond(initial_robot_pose_.rotation());

            std::thread network_thread(&SimplifiedVRController::networkThread, this);

            std::cout << "Waiting for VR data..." << std::endl;
            while (!vr_initialized_ && running_)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (vr_initialized_)
            {
                std::cout << "VR initialized! Starting active control." << std::endl;
                this->runVRControl(robot);
            }

            running_ = false;
            if (network_thread.joinable())
                network_thread.join();
        }
        catch (const franka::Exception &e)
        {
            std::cerr << "Franka exception: " << e.what() << std::endl;
            running_ = false;
        }
    }

private:
    void runVRControl(franka::Robot &robot)
    {
        auto vr_control_callback = [this](
                                       const franka::RobotState &robot_state,
                                       franka::Duration period) -> franka::CartesianPose
        {
            //Update the ultimate goal from VR (~ 50Hz)
            VRCommand cmd;
            {
                std::lock_guard<std::mutex> lock(command_mutex_);
                cmd = current_vr_command_;
            }
            updateVRTargets(cmd);

            // Move the interpolated target a small step towards the ultimate VR target.
            // Define max speeds for the interpolated target
            const double max_step_distance = params_.max_interp_position_step * period.toSec();
            const double max_step_angle = params_.max_interp_orientation_step * period.toSec();

            // Position interpolation
            Eigen::Vector3d pos_error = vr_target_position_ - interpolated_target_position_;
            double distance = pos_error.norm();
            if (distance > 1e-6)
            { // Avoid normalization of zero vector
                double step_distance = std::min(max_step_distance, distance);
                interpolated_target_position_ += (pos_error / distance) * step_distance;
            }

            // Orientation interpolation
            double angle_error = interpolated_target_orientation_.angularDistance(vr_target_orientation_);
            if (angle_error > 1e-6) {
                // Limit the angular step size (same logic as position)
                double step_angle = std::min(max_step_angle, angle_error);
                double slerp_fraction = step_angle / angle_error;
                
                interpolated_target_orientation_ = interpolated_target_orientation_.slerp(slerp_fraction, vr_target_orientation_);
            }

            // The P-controller for the interpolated target.
            Eigen::Map<const Eigen::Matrix4d> pose_map(robot_state.O_T_EE_c.data());
            Eigen::Affine3d current_transform(pose_map);
            Eigen::Vector3d current_position = current_transform.translation();
            Eigen::Quaterniond current_orientation(current_transform.linear());

            const double position_gain = params_.position_gain;
            const double orientation_gain = params_.orientation_gain;

            Eigen::Vector3d next_position = current_position + position_gain * (interpolated_target_position_ - current_position);
            Eigen::Quaterniond next_orientation = current_orientation.slerp(orientation_gain, interpolated_target_orientation_);

            auto pose_array = createPoseArray(next_position, next_orientation);

            if (!running_)
            {
                return franka::MotionFinished(pose_array);
            }
            return pose_array;
        };

        try
        {
            robot.control(vr_control_callback);
        }
        catch (const franka::ControlException &e)
        {
            std::cerr << "VR control exception: " << e.what() << std::endl;
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
        SimplifiedVRController controller;
        // Add a signal handler to gracefully shut down on Ctrl+C
        // std::signal(SIGINT, [](int signum){ controller.stop(); }); // Example, needs a stop() method
        controller.run(argv[1]);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}