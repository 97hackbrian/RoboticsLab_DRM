/**
 * @file visual_odometry_publisher.cpp
 * @brief Visual Odometry Publisher Node for ROS2
 * 
 * Converts camera_link TF to Odometry message for visual odometry tracking.
 * This is a C++ translation of the original Python implementation.
 * 
 * @author Translated from Python to C++ for ROS2 Humble
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

/**
 * @brief Multiply two quaternions
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @return Result quaternion (q1 * q2)
 */
geometry_msgs::msg::Quaternion quaternion_multiply(
    const geometry_msgs::msg::Quaternion& q1,
    const geometry_msgs::msg::Quaternion& q2)
{
    geometry_msgs::msg::Quaternion result;
    
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    
    return result;
}

/**
 * @class VisualOdometryPublisher
 * @brief ROS2 Node that publishes visual odometry based on camera TF transforms
 */
class VisualOdometryPublisher : public rclcpp::Node
{
public:
    /**
     * @brief Constructor - initializes the node with parameters and creates publishers/subscribers
     */
    VisualOdometryPublisher()
        : Node("visual_odometry_publisher"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("camera_frame", "camera_link");
        this->declare_parameter<std::string>("odom_frame", "odom_vo");
        this->declare_parameter<std::string>("child_frame_id", "base_link_vo");
        this->declare_parameter<double>("publish_rate", 30.0);
        
        // Get parameter values
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        child_frame_id_ = this->get_parameter("child_frame_id").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        
        // Initialize TF Broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Initialize Odometry Publisher
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_vo", 10);
        
        // Initialize rotation offset: -90 degrees around Z axis
        // This aligns camera frame with robot base
        // q = [x, y, z, w] for rotation around Z by -90°
        z_rotation_90_.x = 0.0;
        z_rotation_90_.y = 0.0;
        z_rotation_90_.z = -0.7071067811865476;  // sin(-45°)
        z_rotation_90_.w = 0.7071067811865476;   // cos(45°)
        
        // Initialize last error time for throttled logging
        last_error_time_ = this->now();
        
        // Create timer for periodic odometry publishing
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
            std::bind(&VisualOdometryPublisher::publish_odometry, this)
        );
        
        // Log startup information
        RCLCPP_INFO(this->get_logger(), "Visual Odometry Publisher started");
        RCLCPP_INFO(this->get_logger(), "Publishing %s -> %s", 
                    odom_frame_.c_str(), child_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "Using source: %s", camera_frame_.c_str());
    }

private:
    /**
     * @brief Timer callback to publish odometry data
     * 
     * Looks up TF transform from odom_wheel to camera_link and publishes
     * the corresponding visual odometry TF and Odometry message.
     */
    void publish_odometry()
    {
        try {
            // We want to publish odom_vo -> base_link_vo
            // We assume odom_wheel -> camera_link is available from the main tree + sensor
            // And we treat camera_link as the "proxy" for location, but rotated.
            
            // Lookup transform from odom_wheel (the world) to camera_link
            // Note: We use the *simulated* world frame (odom_wheel) to *simulate* the VO result.
            // In a real robot, 'odom_wheel' here would be the output of the VO algorithm relative to its start.
            const std::string source_frame = "odom_wheel";
            
            geometry_msgs::msg::TransformStamped transform;
            transform = tf_buffer_.lookupTransform(
                source_frame,
                camera_frame_,
                tf2::TimePointZero
            );
            
            // --- Prepare Data ---
            // 1. Position: Same as camera
            double pos_x = transform.transform.translation.x;
            double pos_y = transform.transform.translation.y;
            double pos_z = transform.transform.translation.z;
            
            // 2. Orientation: Camera orientation rotated by -90 Z to match robot front
            geometry_msgs::msg::Quaternion q_rot = quaternion_multiply(
                transform.transform.rotation,
                z_rotation_90_
            );
            
            rclcpp::Time current_time = this->now();
            
            // --- 1. Publish TF: odom_vo -> base_link_vo ---
            geometry_msgs::msg::TransformStamped t_vo;
            t_vo.header.stamp = current_time;
            t_vo.header.frame_id = odom_frame_;
            t_vo.child_frame_id = child_frame_id_;
            
            t_vo.transform.translation.x = pos_x;
            t_vo.transform.translation.y = pos_y;
            t_vo.transform.translation.z = pos_z;
            t_vo.transform.rotation = q_rot;
            
            tf_broadcaster_->sendTransform(t_vo);
            
            // --- 2. Publish Odometry Message ---
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = odom_frame_;
            odom.child_frame_id = child_frame_id_;
            
            odom.pose.pose.position.x = pos_x;
            odom.pose.pose.position.y = pos_y;
            odom.pose.pose.position.z = pos_z;
            odom.pose.pose.orientation = q_rot;
            
            // Publish odometry
            odom_pub_->publish(odom);
            
        } catch (const tf2::TransformException& ex) {
            // Throttled logging: only log every 5 seconds to avoid spam
            rclcpp::Time now = this->now();
            if ((now - last_error_time_).seconds() > 5.0) {
                RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
                last_error_time_ = now;
            }
        }
    }
    
    // Member variables
    std::string camera_frame_;
    std::string odom_frame_;
    std::string child_frame_id_;
    
    // TF2 components
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    // Rotation offset quaternion
    geometry_msgs::msg::Quaternion z_rotation_90_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Error throttling
    rclcpp::Time last_error_time_;
};

/**
 * @brief Main entry point
 */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<VisualOdometryPublisher>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
