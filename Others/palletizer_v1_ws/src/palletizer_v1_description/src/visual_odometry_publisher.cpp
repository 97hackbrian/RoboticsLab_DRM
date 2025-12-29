/**
 * @file visual_odometry_publisher.cpp
 * @brief Pseudo Visual Odometry Publisher Node for ROS2
 * 
 * Simulates visual odometry by using ground truth pose from Gazebo.
 * This provides drift-free odometry similar to what an Intel T265 would provide.
 * 
 * Uses TF from /tf_ground_truth which contains world->model_name transforms.
 * 
 * @author Translated from Python to C++ for ROS2 Humble
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <memory>
#include <string>
#include <mutex>

using namespace std::chrono_literals;

/**
 * @class VisualOdometryPublisher
 * @brief ROS2 Node that publishes pseudo visual odometry using ground truth from Gazebo
 * 
 * This node subscribes to /tf_ground_truth which contains world->model transforms
 * from Gazebo's pose/info system. It extracts the robot's pose and republishes it
 * as the odom_vo_frame->base_footprint TF and /odom_vo Odometry message.
 */
class VisualOdometryPublisher : public rclcpp::Node
{
public:
    /**
     * @brief Constructor - initializes the node with parameters and creates publishers/subscribers
     */
    VisualOdometryPublisher()
        : Node("visual_odometry_publisher"),
          pose_received_(false)
    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("model_name", "palletizer_v1");
        this->declare_parameter<std::string>("odom_frame", "odom_vo_frame");
        this->declare_parameter<std::string>("child_frame_id", "base_footprint");
        this->declare_parameter<std::string>("tf_ground_truth_topic", "/tf_ground_truth");
        this->declare_parameter<std::string>("odom_topic", "/odom_vo");
        this->declare_parameter<double>("publish_rate", 50.0);
        
        // Get parameter values
        model_name_ = this->get_parameter("model_name").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        child_frame_id_ = this->get_parameter("child_frame_id").as_string();
        tf_topic_ = this->get_parameter("tf_ground_truth_topic").as_string();
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        
        // Initialize TF Broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Initialize Odometry Publisher
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
        
        // Subscribe to ground truth TF from Gazebo
        // This comes from /world/default/pose/info bridged as TFMessage
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            tf_topic_,
            rclcpp::QoS(10).reliable(),
            std::bind(&VisualOdometryPublisher::tf_callback, this, std::placeholders::_1)
        );
        
        // Create timer for periodic TF/Odom publishing
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
            std::bind(&VisualOdometryPublisher::publish_odometry, this)
        );
        
        // Log startup information
        RCLCPP_INFO(this->get_logger(), "Pseudo Visual Odometry Publisher started");
        RCLCPP_INFO(this->get_logger(), "Listening for model '%s' on %s", 
                    model_name_.c_str(), tf_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing TF: %s -> %s", 
                    odom_frame_.c_str(), child_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing odometry on: %s", odom_topic_.c_str());
    }

private:
    /**
     * @brief Callback for TF messages from ground truth
     * @param msg The TFMessage containing all model poses
     */
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        
        // Search for our model in the TF message
        for (const auto& transform : msg->transforms) {
            if (transform.child_frame_id == model_name_) {
                latest_transform_ = transform;
                pose_received_ = true;
                return;
            }
        }
    }
    
    /**
     * @brief Timer callback to publish odometry data
     * 
     * Uses the latest ground truth pose to publish TF and Odometry.
     */
    void publish_odometry()
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        
        if (!pose_received_) {
            // Only log occasionally to avoid spam
            static int wait_count = 0;
            if (++wait_count % 100 == 1) {
                RCLCPP_WARN(this->get_logger(), 
                    "Waiting for ground truth TF for model '%s' on %s...", 
                    model_name_.c_str(), tf_topic_.c_str());
            }
            return;
        }
        
        rclcpp::Time current_time = this->now();
        
        // Extract position from ground truth
        double pos_x = latest_transform_.transform.translation.x;
        double pos_y = latest_transform_.transform.translation.y;
        double pos_z = latest_transform_.transform.translation.z;
        
        // Extract orientation from ground truth
        geometry_msgs::msg::Quaternion orientation = latest_transform_.transform.rotation;
        
        // --- 1. Publish TF: odom -> base_footprint ---
        geometry_msgs::msg::TransformStamped t_odom;
        t_odom.header.stamp = current_time;
        t_odom.header.frame_id = odom_frame_;
        t_odom.child_frame_id = child_frame_id_;
        
        t_odom.transform.translation.x = pos_x;
        t_odom.transform.translation.y = pos_y;
        t_odom.transform.translation.z = pos_z;
        t_odom.transform.rotation = orientation;
        
        tf_broadcaster_->sendTransform(t_odom);
        
        // --- 2. Publish Odometry Message ---
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = child_frame_id_;
        
        odom.pose.pose.position.x = pos_x;
        odom.pose.pose.position.y = pos_y;
        odom.pose.pose.position.z = pos_z;
        odom.pose.pose.orientation = orientation;
        
        // Set covariance (very low since this is ground truth)
        // Position covariance
        odom.pose.covariance[0] = 0.001;   // x
        odom.pose.covariance[7] = 0.001;   // y
        odom.pose.covariance[14] = 0.001;  // z
        // Orientation covariance
        odom.pose.covariance[21] = 0.001;  // roll
        odom.pose.covariance[28] = 0.001;  // pitch
        odom.pose.covariance[35] = 0.001;  // yaw
        
        // Publish odometry
        odom_pub_->publish(odom);
    }
    
    // Member variables
    std::string model_name_;
    std::string odom_frame_;
    std::string child_frame_id_;
    std::string tf_topic_;
    std::string odom_topic_;
    
    // Latest transform data
    geometry_msgs::msg::TransformStamped latest_transform_;
    bool pose_received_;
    std::mutex pose_mutex_;
    
    // TF Broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Publishers and Subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
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
