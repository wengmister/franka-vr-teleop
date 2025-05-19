#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/callback_group.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h> 

#include <thread>
#include <mutex>
#include <regex>
#include <vector>
#include <string>
#include <chrono>
#include <cmath> // For std::isnan

using namespace std::chrono_literals;

class WristServoPoseControlCpp : public rclcpp::Node
{
public:
    WristServoPoseControlCpp()
    : Node("wrist_servo_pose_control_cpp"),
      udp_socket_(-1),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        // Parameters
        this->declare_parameter<int>("udp_port", 9000);
        this->declare_parameter<std::string>("udp_ip", "0.0.0.0");
        this->declare_parameter<std::string>("planning_frame", "fer_link0");
        this->declare_parameter<std::string>("ee_frame", "fer_link8");
        this->declare_parameter<double>("pose_change_scale", 0.6);
        this->declare_parameter<double>("max_position_change", 0.06);
        this->declare_parameter<double>("max_orientation_change", 0.08);
        this->declare_parameter<double>("position_smoothing", 0.75);
        this->declare_parameter<double>("orientation_smoothing", 0.75);

        udp_port_ = this->get_parameter("udp_port").as_int();
        udp_ip_ = this->get_parameter("udp_ip").as_string();
        planning_frame_ = this->get_parameter("planning_frame").as_string();
        ee_frame_ = this->get_parameter("ee_frame").as_string();
        pose_change_scale_ = this->get_parameter("pose_change_scale").as_double();
        max_position_change_ = this->get_parameter("max_position_change").as_double();
        max_orientation_change_ = this->get_parameter("max_orientation_change").as_double();
        position_smoothing_ = this->get_parameter("position_smoothing").as_double();
        orientation_smoothing_ = this->get_parameter("orientation_smoothing").as_double();

        RCLCPP_INFO(this->get_logger(), "Parameters loaded: ");
        RCLCPP_INFO(this->get_logger(), "  udp_port: %d", udp_port_);
        RCLCPP_INFO(this->get_logger(), "  udp_ip: %s", udp_ip_.c_str());
        RCLCPP_INFO(this->get_logger(), "  planning_frame: %s", planning_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  ee_frame: %s", ee_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  pose_change_scale: %.2f", pose_change_scale_);
        RCLCPP_INFO(this->get_logger(), "  max_position_change: %.2f", max_position_change_);
        RCLCPP_INFO(this->get_logger(), "  max_orientation_change: %.2f", max_orientation_change_);
        RCLCPP_INFO(this->get_logger(), "  position_smoothing: %.2f", position_smoothing_);
        RCLCPP_INFO(this->get_logger(), "  orientation_smoothing: %.2f", orientation_smoothing_);


        // Callback group for services (can be reentrant if services are called from multiple threads/timers)
        service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create a publisher for pose commands with reliable QoS
        auto pose_qos = rclcpp::QoS(10).reliable();
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/servo_node/pose_target", pose_qos);
        
        // For visualization and debugging with best effort QoS
        auto viz_qos = rclcpp::QoS(10).best_effort();
        wrist_viz_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/wrist_pose_cpp", viz_qos);

        switch_command_client_ = this->create_client<moveit_msgs::srv::ServoCommandType>(
            "/servo_node/switch_command_type", rmw_qos_profile_services_default, service_callback_group_);
        start_pose_client_ = this->create_client<std_srvs::srv::Trigger>(
            "/servo_node/start_pose_command", rmw_qos_profile_services_default, service_callback_group_);

        while (!switch_command_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the switch_command_type service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for switch_command_type service...");
        }
        initialize_command_type();

        // Initialize poses
        prev_wrist_position_.setZero();
        prev_wrist_orientation_.setIdentity();
        smoothed_position_delta_.setZero();
        smoothed_orientation_delta_axis_angle_.setZero(); // Axis-angle

        setup_udp_socket();
        if (udp_socket_ != -1) {
            udp_thread_ = std::thread(&WristServoPoseControlCpp::receive_udp_messages_loop, this);
        }

        // Main control loop timer - was 50 Hz, increasing to 100 Hz for smoother motion
        control_timer_ = this->create_wall_timer(
            10ms, std::bind(&WristServoPoseControlCpp::pose_control_loop, this)); // 100 Hz

        RCLCPP_INFO(this->get_logger(), "Wrist Servo Pose Control C++ node started. Listening on %s:%d", udp_ip_.c_str(), udp_port_);
    }

    ~WristServoPoseControlCpp()
    {
        if (udp_socket_ != -1) {
            close(udp_socket_);
        }
        if (udp_thread_.joinable()) {
            udp_thread_.join();
        }
    }

private:
    void initialize_command_type()
    {
        auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
        request->command_type = moveit_msgs::srv::ServoCommandType::Request::POSE;
        
        RCLCPP_INFO(this->get_logger(), "Setting command type to POSE...");
        auto future_switch = switch_command_client_->async_send_request(request);
        // Minimal wait, primarily for demonstration. Proper handling would involve checking future status.
        std::future_status status_switch = future_switch.wait_for(2s);
        if (status_switch == std::future_status::ready) {
             RCLCPP_INFO(this->get_logger(), "Switch command type request sent.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to send switch command type request or timed out.");
        }


        // Check if start_pose_command service exists (optional, but good practice)
        // Note: get_service_names_and_types is not directly available like in Python to check beforehand.
        // We'll just try to call it.
        if (start_pose_client_->service_is_ready()) { // Or simply try calling if you expect it
            RCLCPP_INFO(this->get_logger(), "Starting pose command mode via trigger service...");
            auto start_request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto future_start = start_pose_client_->async_send_request(start_request);
            std::future_status status_start = future_start.wait_for(2s);
             if (status_start == std::future_status::ready) {
                RCLCPP_INFO(this->get_logger(), "Start pose command mode request sent.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to send start pose command mode request or timed out.");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Start_pose_command service not immediately available.");
        }
    }

    bool get_current_ee_pose(Eigen::Isometry3d& ee_pose)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(
                planning_frame_, ee_frame_, tf2::TimePointZero); // Get latest
            ee_pose = tf2::transformToEigen(transform_stamped);
            return true;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not get EE transform: %s", ex.what());
            return false;
        }
    }

    void setup_udp_socket()
    {
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket.");
            return;
        }

        sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = inet_addr(udp_ip_.c_str()); // Use configured IP or INADDR_ANY for "0.0.0.0"
        if (udp_ip_ == "0.0.0.0") {
            server_addr.sin_addr.s_addr = INADDR_ANY;
        }
        server_addr.sin_port = htons(udp_port_);

        if (bind(udp_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind UDP socket to %s:%d. Error: %s", udp_ip_.c_str(), udp_port_, strerror(errno));
            close(udp_socket_);
            udp_socket_ = -1;
            return;
        }
        
        // Set non-blocking
        int flags = fcntl(udp_socket_, F_GETFL, 0);
        if (flags == -1) {
             RCLCPP_ERROR(this->get_logger(), "Failed to get socket flags. Error: %s", strerror(errno));
             close(udp_socket_);
             udp_socket_ = -1;
             return;
        }
        if (fcntl(udp_socket_, F_SETFL, flags | O_NONBLOCK) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set socket to non-blocking. Error: %s", strerror(errno));
            close(udp_socket_);
            udp_socket_ = -1;
        }
    }

    void receive_udp_messages_loop()
    {
        char buffer[1024];
        RCLCPP_INFO(this->get_logger(), "UDP receiver thread started.");
        while (rclcpp::ok() && udp_socket_ != -1) {
            ssize_t len = recvfrom(udp_socket_, buffer, sizeof(buffer) -1, 0, nullptr, nullptr);
            if (len > 0) {
                buffer[len] = '\0'; // Null-terminate
                parse_udp_message(std::string(buffer));
            } else if (len < 0) {
                if (errno != EWOULDBLOCK && errno != EAGAIN) {
                    RCLCPP_ERROR(this->get_logger(), "UDP recvfrom error: %s", strerror(errno));
                }
                // No data, sleep briefly
                std::this_thread::sleep_for(1ms);
            } else { // len == 0, connection closed by peer (not typical for UDP)
                 std::this_thread::sleep_for(1ms);
            }
        }
        RCLCPP_INFO(this->get_logger(), "UDP receiver thread stopped.");
    }
    
    std::regex wrist_pattern_{R"(Right wrist:, ([-\d\.]+), ([-\d\.]+), ([-\d\.]+), ([-\d\.]+), ([-\d\.]+), ([-\d\.]+), ([-\d\.]+))"};

    void parse_udp_message(const std::string& message)
    {
        std::smatch match;
        if (std::regex_search(message, match, wrist_pattern_) && match.size() == 8) {
            try {
                double x = std::stod(match[1].str());
                double y = std::stod(match[2].str());
                double z = std::stod(match[3].str());
                double qx = std::stod(match[4].str());
                double qy = std::stod(match[5].str());
                double qz = std::stod(match[6].str());
                double qw = std::stod(match[7].str());

                // Transform: input +x = output -y, input +y = output +z, input +z = output +x
                Eigen::Vector3d transformed_position(z, -x, y);
                
                Eigen::Quaterniond input_quat(qw, qx, qy, qz); // Eigen constructor is w,x,y,z
                input_quat.normalize();

                // Coord transform: 90° around X, then -90° around new Z
                Eigen::Quaterniond coord_transform_x(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX()));
                Eigen::Quaterniond coord_transform_z(Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ()));
                Eigen::Quaterniond coord_transform = coord_transform_z * coord_transform_x; // Order matters
                
                Eigen::Quaterniond output_orientation = coord_transform * input_quat;
                output_orientation.normalize();

                publish_wrist_viz_pose(transformed_position, output_orientation);

                std::lock_guard<std::mutex> lock(pose_data_mutex_);
                if (prev_wrist_position_initialized_) {
                    Eigen::Vector3d pos_delta = transformed_position - prev_wrist_position_;
                    
                    Eigen::Quaterniond rel_orientation_quat = output_orientation * prev_wrist_orientation_.inverse();
                    rel_orientation_quat.normalize();
                    Eigen::AngleAxisd rel_orientation_aa(rel_orientation_quat); // Axis-angle

                    Eigen::Vector3d scaled_pos_delta = pos_delta * pose_change_scale_;
                    Eigen::Vector3d scaled_orientation_aa_vec = rel_orientation_aa.axis() * rel_orientation_aa.angle() * pose_change_scale_;
                    
                    // Check for NaN/inf before smoothing
                    if (!scaled_pos_delta.hasNaN() && !scaled_orientation_aa_vec.hasNaN()) {
                         smoothed_position_delta_ = position_smoothing_ * smoothed_position_delta_ + 
                                               (1.0 - position_smoothing_) * scaled_pos_delta;
                        smoothed_orientation_delta_axis_angle_ = orientation_smoothing_ * smoothed_orientation_delta_axis_angle_ +
                                                            (1.0 - orientation_smoothing_) * scaled_orientation_aa_vec;
                    } else {
                        RCLCPP_WARN(this->get_logger(), "NaN detected in scaled deltas, skipping smoothing update.");
                    }
                }

                prev_wrist_position_ = transformed_position;
                prev_wrist_orientation_ = output_orientation;
                prev_wrist_position_initialized_ = true;

            } catch (const std::invalid_argument& ia) {
                RCLCPP_WARN(this->get_logger(), "Invalid argument in UDP message parsing: %s. Message: %s", ia.what(), message.c_str());
            } catch (const std::out_of_range& oor) {
                RCLCPP_WARN(this->get_logger(), "Out of range in UDP message parsing: %s. Message: %s", oor.what(), message.c_str());
            }
        }
    }

    void publish_wrist_viz_pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = planning_frame_; // Assuming visualization in planning frame
        pose_msg.pose.position = tf2::toMsg(position);
        pose_msg.pose.orientation = tf2::toMsg(orientation);
        wrist_viz_pub_->publish(pose_msg);
    }

    void pose_control_loop()
    {
        if (!current_ee_pose_initialized_) {
            Eigen::Isometry3d current_pose_eigen;
            if (get_current_ee_pose(current_pose_eigen)) {
                current_ee_pose_ = current_pose_eigen;
                current_ee_pose_initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "Got initial EE pose.");
            } else {
                return; // Wait for TF
            }
        }
        
        if (!prev_wrist_position_initialized_) {
            return; // Wait for first UDP message
        }

        geometry_msgs::msg::PoseStamped target_pose_msg;
        target_pose_msg.header.frame_id = planning_frame_;

        Eigen::Vector3d pos_delta_to_apply;
        Eigen::Vector3d rot_delta_axis_angle_to_apply;
        {
            std::lock_guard<std::mutex> lock(pose_data_mutex_);
            pos_delta_to_apply = smoothed_position_delta_;
            rot_delta_axis_angle_to_apply = smoothed_orientation_delta_axis_angle_;
        }
        
        // Limit magnitude of change
        if (pos_delta_to_apply.norm() > max_position_change_) {
            pos_delta_to_apply.normalize();
            pos_delta_to_apply *= max_position_change_;
        }
        if (rot_delta_axis_angle_to_apply.norm() > max_orientation_change_) {
             // Normalize axis * angle vector means normalizing the angle component basically
            rot_delta_axis_angle_to_apply.normalize();
            rot_delta_axis_angle_to_apply *= max_orientation_change_;
        }
        
        // Apply deltas to current_ee_pose_
        current_ee_pose_.translation() += pos_delta_to_apply;
        
        if (rot_delta_axis_angle_to_apply.norm() > 1e-6) { // Only apply if meaningful rotation
            double angle = rot_delta_axis_angle_to_apply.norm();
            Eigen::Vector3d axis = rot_delta_axis_angle_to_apply.normalized();
            Eigen::Quaterniond incremental_rotation(Eigen::AngleAxisd(angle, axis));
            current_ee_pose_.rotate(incremental_rotation);
        }

        target_pose_msg.pose = tf2::toMsg(current_ee_pose_);
        
        // Add explicit timestamp to ensure message freshness
        target_pose_msg.header.stamp = this->get_clock()->now();
        
        // Publish and ensure message is processed
        pose_pub_->publish(target_pose_msg);
        
        // Log the target pose being sent
        RCLCPP_INFO(this->get_logger(), "Sending pose command: position=[%.4f, %.4f, %.4f], orientation=[%.4f, %.4f, %.4f, %.4f]",
                    target_pose_msg.pose.position.x, target_pose_msg.pose.position.y, target_pose_msg.pose.position.z,
                    target_pose_msg.pose.orientation.x, target_pose_msg.pose.orientation.y, target_pose_msg.pose.orientation.z, target_pose_msg.pose.orientation.w);
                    
        // Ensure the message is sent with high QoS
        // No need to call spin_some() as node is already managed by executor
    }

    // Member Variables
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr wrist_viz_pub_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_command_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_pose_client_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    int udp_socket_;
    std::thread udp_thread_;
    std::mutex pose_data_mutex_; // Protects smoothed_deltas and prev_wrist_data if accessed by multiple threads

    // Parameters
    int udp_port_;
    std::string udp_ip_;
    std::string planning_frame_;
    std::string ee_frame_;
    double pose_change_scale_;
    double max_position_change_;
    double max_orientation_change_;
    double position_smoothing_;
    double orientation_smoothing_;
    
    // State
    Eigen::Vector3d prev_wrist_position_;
    Eigen::Quaterniond prev_wrist_orientation_;
    bool prev_wrist_position_initialized_ = false;

    Eigen::Isometry3d current_ee_pose_;
    bool current_ee_pose_initialized_ = false;

    Eigen::Vector3d smoothed_position_delta_;
    Eigen::Vector3d smoothed_orientation_delta_axis_angle_; // Store as axis-angle
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WristServoPoseControlCpp>();
    // Using MultiThreadedExecutor for TF, service clients, and potentially multiple timers/subscriptions
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
