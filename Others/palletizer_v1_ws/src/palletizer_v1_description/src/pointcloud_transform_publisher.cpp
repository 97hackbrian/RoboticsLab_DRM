/**
 * @file pointcloud_transform_publisher.cpp
 * @brief Point Cloud Transform Publisher Node for ROS2
 * 
 * This node subscribes to a point cloud and republishes it with
 * the points transformed from optical frame to robot frame.
 * 
 * The stereo camera optical frame convention is:
 * - Z forward (depth)
 * - X right
 * - Y down
 * 
 * The camera_link/robot frame convention is:
 * - X forward
 * - Y left
 * - Z up
 * 
 * Transform from optical to camera_link (based on TF):
 * x_new = z_optical
 * y_new = -x_optical
 * z_new = -y_optical
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <cstring>

class PointCloudTransformPublisher : public rclcpp::Node
{
public:
    PointCloudTransformPublisher()
        : Node("pointcloud_transform_publisher")
    {
        // Declare parameters
        this->declare_parameter<std::string>("input_topic", "/camera/depth/points");
        this->declare_parameter<std::string>("output_topic", "/camera/depth/points_transformed");
        this->declare_parameter<std::string>("output_frame_id", "camera_link");
        
        // Get parameters
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        output_frame_id_ = this->get_parameter("output_frame_id").as_string();
        
        // QoS profile - match the input
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        
        // Publisher with reliable QoS for visualization
        rclcpp::QoS pub_qos(10);
        pub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        
        // Publisher
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, pub_qos);
        
        // Subscriber
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, qos,
            std::bind(&PointCloudTransformPublisher::callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "PointCloud Transform Publisher started");
        RCLCPP_INFO(this->get_logger(), "Input:  %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Output: %s -> frame: %s", output_topic.c_str(), output_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "Transform: optical -> camera_link");
        RCLCPP_INFO(this->get_logger(), "  x_new = z_optical (depth -> forward)");
        RCLCPP_INFO(this->get_logger(), "  y_new = -x_optical (right -> left)");
        RCLCPP_INFO(this->get_logger(), "  z_new = -y_optical (down -> up)");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (msg->data.empty()) {
            return;
        }
        
        // Create output message as a copy
        auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();
        output->header = msg->header;
        output->header.frame_id = output_frame_id_;
        output->height = msg->height;
        output->width = msg->width;
        output->fields = msg->fields;
        output->is_bigendian = msg->is_bigendian;
        output->point_step = msg->point_step;
        output->row_step = msg->row_step;
        output->is_dense = msg->is_dense;
        output->data.resize(msg->data.size());
        
        // Copy the data first
        std::memcpy(output->data.data(), msg->data.data(), msg->data.size());
        
        // Find xyz field offsets
        int x_offset = -1, y_offset = -1, z_offset = -1;
        
        for (const auto& field : msg->fields) {
            if (field.name == "x") { x_offset = field.offset; }
            else if (field.name == "y") { y_offset = field.offset; }
            else if (field.name == "z") { z_offset = field.offset; }
        }
        
        if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Point cloud does not have xyz fields");
            pub_->publish(*output);
            return;
        }
        
        // Transform each point
        // From optical frame to camera_link:
        // x_new = z_optical
        // y_new = -x_optical
        // z_new = -y_optical
        for (size_t i = 0; i < msg->width * msg->height; ++i) {
            size_t point_offset = i * msg->point_step;
            
            // Read original coordinates from input
            float x_opt, y_opt, z_opt;
            std::memcpy(&x_opt, &msg->data[point_offset + x_offset], sizeof(float));
            std::memcpy(&y_opt, &msg->data[point_offset + y_offset], sizeof(float));
            std::memcpy(&z_opt, &msg->data[point_offset + z_offset], sizeof(float));
            
            // Skip NaN points
            if (std::isnan(x_opt) || std::isnan(y_opt) || std::isnan(z_opt)) {
                continue;
            }
            
            // Apply transformation: optical -> camera_link
            float x_new = z_opt;      // depth becomes forward
            float y_new = -x_opt;     // right becomes left (negate)
            float z_new = -y_opt;     // down becomes up (negate)
            
            // Write transformed coordinates to output
            std::memcpy(&output->data[point_offset + x_offset], &x_new, sizeof(float));
            std::memcpy(&output->data[point_offset + y_offset], &y_new, sizeof(float));
            std::memcpy(&output->data[point_offset + z_offset], &z_new, sizeof(float));
        }
        
        pub_->publish(*output);
    }
    
    // Output frame ID
    std::string output_frame_id_;
    
    // Publisher and Subscriber
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudTransformPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
