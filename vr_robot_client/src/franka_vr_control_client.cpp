// VR-Based Cartesian Teleoperation - Simplified with Callback Control
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

    // Simplified parameters - no trajectory generation needed
    struct VRParams
    {
        // VR mapping parameters
        double position_responsiveness = 0.5;    // How much robot moves per VR movement
        double orientation_responsiveness = 0.5; // How much robot rotates per VR rotation
        double vr_smoothing = 0.8;               // Smoothing of incoming VR data

        // Target smoothing - smooth target changes
        double target_position_smoothing = 0.9;
        double target_orientation_smoothing = 0.9;

        // Deadzones
        double position_deadzone = 0.002;   // 2mm
        double orientation_deadzone = 0.03; // ~1.7 degrees

        // Workspace limits
        double max_position_offset = 0.15;    // 15cm from initial position
        double max_orientation_offset = 0.25; // ~14 degrees from initial orientation
    } params_;

    // Current targets
    Eigen::Vector3d target_position_;
    Eigen::Quaterniond target_orientation_;

    // Smoothed targets to prevent sudden jumps
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
        Eigen::Vector3d raw_target_position = initial_robot_pose_.translation() + 
                                            vr_pos_delta * params_.position_responsiveness;
        Eigen::Quaterniond raw_target_orientation = vr_quat_delta * Eigen::Quaterniond(initial_robot_pose_.rotation());
        raw_target_orientation.normalize();

        // Smooth target changes to prevent sudden jumps
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

    std::array<double, 16> createPoseArray(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
    {
        Eigen::Affine3d pose;
        pose.translation() = position;
        pose.linear() = orientation.toRotationMatrix();
        
        std::array<double, 16> pose_array;
        Eigen::Map<Eigen::Matrix4d>(pose_array.data()) = pose.matrix();
        return pose_array;
    }

    void initializeSmoothedTargets()
    {
        smoothed_target_position_ = target_position_;
        smoothed_target_orientation_ = target_orientation_;
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

            // Set collision behavior - more conservative than example
            robot.setCollisionBehavior(
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

            // Initialize from current pose
            franka::RobotState state = robot.readOnce();
            initial_robot_pose_ = Eigen::Affine3d(Eigen::Matrix4d::Map(state.O_T_EE_d.data()));

            target_position_ = initial_robot_pose_.translation();
            target_orientation_ = Eigen::Quaterniond(initial_robot_pose_.rotation());

            // Initialize smoothed targets
            initializeSmoothedTargets();

            std::thread network_thread(&SimplifiedVRController::networkThread, this);

            std::cout << "Waiting for VR data..." << std::endl;
            while (!vr_initialized_ && running_)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (vr_initialized_)
            {
                std::cout << "VR initialized! Starting active control..." << std::endl;
                this->runVRControl(robot);
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
    void runVRControl(franka::Robot &robot)
    {
        std::cout << "Starting VR control..." << std::endl;

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

        // Callback-based control (simpler approach)
        auto vr_control_callback = [this, &iteration_count](
                                      const franka::RobotState& robot_state,
                                      franka::Duration period) -> franka::CartesianPose {
            iteration_count++;

            // Get current VR command
            VRCommand cmd;
            {
                std::lock_guard<std::mutex> lock(command_mutex_);
                cmd = current_vr_command_;
            }

            // Update VR targets
            if (vr_initialized_)
            {
                updateVRTargets(cmd);
            }

            // Debug output
            if (iteration_count % 1000 == 0)
            {
                Eigen::Vector3d current_pos = Eigen::Vector3d::Map(robot_state.O_T_EE_c.data() + 12);
                Eigen::Vector3d pos_error = target_position_ - current_pos;
                
                std::cout << "VR Control:" << std::endl
                          << "  Target pos: [" << target_position_.x() << ", " 
                          << target_position_.y() << ", " << target_position_.z() << "]" << std::endl
                          << "  Current pos: [" << current_pos.x() << ", " 
                          << current_pos.y() << ", " << current_pos.z() << "]" << std::endl
                          << "  Position error: " << pos_error.norm() << " m" << std::endl;
            }

            // Create and return pose array from current targets
            auto pose_array = createPoseArray(target_position_, target_orientation_);

            // Check if we should stop (you can add conditions here)
            if (!running_)
            {
                return franka::MotionFinished(pose_array);
            }

            return pose_array;
        };

        try
        {
            std::cout << "VR control started. Use VR to control the robot. Press Ctrl+C to stop." << std::endl;
            robot.control(vr_control_callback);
            std::cout << "VR control finished." << std::endl;
        }
        catch (const franka::ControlException &e)
        {
            std::cout << "VR control exception: " << e.what() << std::endl;
        }
        catch (const franka::Exception &e)
        {
            std::cout << "Franka exception during VR control: " << e.what() << std::endl;
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
        controller.run(argv[1]);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}