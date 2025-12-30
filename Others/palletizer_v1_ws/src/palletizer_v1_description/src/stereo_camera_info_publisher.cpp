/**
 * @file stereo_camera_info_publisher.cpp
 * @brief Stereo Camera Info Publisher Node for ROS2
 * 
 * This node subscribes to the original camera_info and republishes it with
 * the correct baseline information for stereo processing.
 * 
 * For stereo cameras, the right camera's CameraInfo P matrix needs to include
 * the baseline (Tx) in P[0,3] = -fx * baseline
 * 
 * Parameters:
 * - baseline: Distance between left and right cameras in meters (default: 0.16)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <memory>
#include <string>

class StereoCameraInfoPublisher : public rclcpp::Node
{
public:
    StereoCameraInfoPublisher()
        : Node("stereo_camera_info_publisher")
    {
        // Declare parameters
        this->declare_parameter<double>("baseline", 0.16);
        this->declare_parameter<std::string>("left_camera_info_in", "/camera/left/camera_info");
        this->declare_parameter<std::string>("right_camera_info_in", "/camera/right/camera_info");
        this->declare_parameter<std::string>("left_camera_info_out", "/stereo/left/camera_info");
        this->declare_parameter<std::string>("right_camera_info_out", "/stereo/right/camera_info");
        this->declare_parameter<std::string>("left_frame_id", "camera_left_optical_frame");
        this->declare_parameter<std::string>("right_frame_id", "camera_right_optical_frame");
        
        // Get parameters
        baseline_ = this->get_parameter("baseline").as_double();
        std::string left_in = this->get_parameter("left_camera_info_in").as_string();
        std::string right_in = this->get_parameter("right_camera_info_in").as_string();
        std::string left_out = this->get_parameter("left_camera_info_out").as_string();
        std::string right_out = this->get_parameter("right_camera_info_out").as_string();
        left_frame_id_ = this->get_parameter("left_frame_id").as_string();
        right_frame_id_ = this->get_parameter("right_frame_id").as_string();
        
        // QoS profile
        rclcpp::QoS qos(10);
        qos.reliable();
        
        // Publishers
        left_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(left_out, qos);
        right_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(right_out, qos);
        
        // Subscribers
        left_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            left_in, qos,
            std::bind(&StereoCameraInfoPublisher::left_callback, this, std::placeholders::_1));
        
        right_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            right_in, qos,
            std::bind(&StereoCameraInfoPublisher::right_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Stereo Camera Info Publisher started");
        RCLCPP_INFO(this->get_logger(), "Baseline: %.3f m", baseline_);
        RCLCPP_INFO(this->get_logger(), "Left:  %s -> %s", left_in.c_str(), left_out.c_str());
        RCLCPP_INFO(this->get_logger(), "Right: %s -> %s", right_in.c_str(), right_out.c_str());
    }

private:
    void left_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // Create modified message
        auto new_msg = sensor_msgs::msg::CameraInfo(*msg);
        new_msg.header.frame_id = left_frame_id_;
        
        // Left camera P matrix is unchanged
        left_pub_->publish(new_msg);
    }
    
    void right_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // Get fx from P matrix
        double fx = msg->p[0];
        
        // Calculate Tx = -fx * baseline
        double tx = -fx * baseline_;
        
        // Create modified message
        auto new_msg = sensor_msgs::msg::CameraInfo(*msg);
        new_msg.header.frame_id = right_frame_id_;
        
        // Modify P matrix for right camera
        // P = [fx  0  cx Tx]
        //     [0  fy  cy  0]
        //     [0   0   1  0]
        // For right camera, P[3] = Tx = -fx * baseline
        new_msg.p[3] = tx;
        
        right_pub_->publish(new_msg);
    }
    
    // Parameters
    double baseline_;
    std::string left_frame_id_;
    std::string right_frame_id_;
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoCameraInfoPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
