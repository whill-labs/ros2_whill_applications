// Copyright (c) 2024 WHILL, Inc.
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class LaserScanFilterNode : public rclcpp::Node {
public:
    LaserScanFilterNode() : Node("laserscan_filter_node"), tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)), tf_listener_(tf_buffer_) {
        // Declare and get parameters
        this->declare_parameter<double>("forward.x_min", -1.0);
        this->declare_parameter<double>("forward.x_max", 1.0);
        this->declare_parameter<double>("forward.y_min", -1.0);
        this->declare_parameter<double>("forward.y_max", 1.0);
        this->declare_parameter<double>("backward.x_min", -1.0);
        this->declare_parameter<double>("backward.x_max", 1.0);
        this->declare_parameter<double>("backward.y_min", -1.0);
        this->declare_parameter<double>("backward.y_max", 1.0);
        this->declare_parameter<std::string>("reference_link", "base_link");

        this->get_parameter("reference_link", reference_link_);
        this->get_parameter("forward.x_min", forward_x_min_);
        this->get_parameter("forward.x_max", forward_x_max_);
        this->get_parameter("forward.y_min", forward_y_min_);
        this->get_parameter("forward.y_max", forward_y_max_);
        this->get_parameter("backward.x_min", backward_x_min_);
        this->get_parameter("backward.x_max", backward_x_max_);
        this->get_parameter("backward.y_min", backward_y_min_);
        this->get_parameter("backward.y_max", backward_y_max_);

        // Subscriber for LaserScan messages
        laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&LaserScanFilterNode::laserScanCallback, this, std::placeholders::_1));

        // Subscriber for velocity messages
        velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&LaserScanFilterNode::velocityCallback, this, std::placeholders::_1));

        // Publisher for Bool messages
        bool_publisher_ = this->create_publisher<std_msgs::msg::Bool>("scan_in_range", 10);

        // Publisher for detected points
        detected_points_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("detected_points", 10);

        // Publisher for polygon visualization
        polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("scan_range_polygon", 10);

        RCLCPP_INFO(this->get_logger(), "Laser Scan Filter Node has been started.");
        RCLCPP_INFO(this->get_logger(), "Reference link is set to: %s", reference_link_.c_str());

    }

private:
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &quat_msg)
    {
        tf2::Quaternion quat;
        tf2::fromMsg(quat_msg, quat);

        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        return yaw;
    }

    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Check the linear x velocity to determine forward or bakward direction
        is_forward_ = (msg->linear.x >= 0);
        linear_x_ = msg->linear.x;
        RCLCPP_DEBUG(this->get_logger(), "Direction: %s", is_forward_ ? "Forward" : "Backward");
    }

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        bool in_range = false;

        // Select appropriate bounds based on direction
        double x_min = is_forward_ ? forward_x_min_ : backward_x_min_ + linear_x_;
        double x_max = is_forward_ ? forward_x_max_ + linear_x_ : backward_x_max_;
        double y_min = is_forward_ ? forward_y_min_ : backward_y_min_;
        double y_max = is_forward_ ? forward_y_max_ : backward_y_max_;

        publishPolygon(x_min, x_max, y_min, y_max);

        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(reference_link_, msg->header.frame_id, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", msg->header.frame_id.c_str(), reference_link_.c_str(), ex.what());
            return;
        }

        auto detected_points_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
        *detected_points_msg = *msg;
        detected_points_msg->ranges.clear();
        detected_points_msg->intensities.clear();

        // Iterate over the LaserScan ranges
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            // Skip invalid ranges
            if (std::isnan(msg->ranges[i]) || std::isinf(msg->ranges[i])) {
                detected_points_msg->ranges.push_back(std::numeric_limits<float>::quiet_NaN());
                detected_points_msg->intensities.push_back(0.0);
                continue;
            }

            // Compute angle and position in local frame
            double angle = msg->angle_min + i * msg->angle_increment;
            double x_local = msg->ranges[i] * std::cos(angle);
            double y_local = msg->ranges[i] * std::sin(angle);

            // Get yaw angle from quaternion
            double yaw = getYawFromQuaternion(transform_stamped.transform.rotation);

            // Transform to reference frame
            double x_ref = transform_stamped.transform.translation.x +
                        x_local * std::cos(yaw) - y_local * std::sin(yaw);
            double y_ref = transform_stamped.transform.translation.y +
                        x_local * std::sin(yaw) + y_local * std::cos(yaw);

            // Check if the point is within the defined range
            if (x_ref >= x_min && x_ref <= x_max && y_ref >= y_min && y_ref <= y_max) {
                in_range = true;
                detected_points_msg->ranges.push_back(msg->ranges[i]);
                detected_points_msg->intensities.push_back(msg->intensities[i]);
            } else {
                detected_points_msg->ranges.push_back(std::numeric_limits<float>::quiet_NaN());
                detected_points_msg->intensities.push_back(0.0);
            }
        }

        // Publish the result
        std_msgs::msg::Bool bool_msg;
        bool_msg.data = in_range;
        bool_publisher_->publish(bool_msg);

        detected_points_publisher_->publish(*detected_points_msg);

        RCLCPP_INFO(this->get_logger(), "Scan in range: %s", in_range ? "True" : "False");
    }

    void publishPolygon(double x_min, double x_max, double y_min, double y_max) {
        auto polygon_msg = geometry_msgs::msg::PolygonStamped();
        polygon_msg.header.frame_id = reference_link_;
        polygon_msg.header.stamp = this->now();

        geometry_msgs::msg::Point32 p1, p2, p3, p4;

        // Define the corners of the rectangle
        p1.x = x_min; p1.y = y_min; p1.z = 0.0;
        p2.x = x_max; p2.y = y_min; p2.z = 0.0;
        p3.x = x_max; p3.y = y_max; p3.z = 0.0;
        p4.x = x_min; p4.y = y_max; p4.z = 0.0;

        polygon_msg.polygon.points.push_back(p1);
        polygon_msg.polygon.points.push_back(p2);
        polygon_msg.polygon.points.push_back(p3);
        polygon_msg.polygon.points.push_back(p4);
        polygon_msg.polygon.points.push_back(p1); // Close the loop

        polygon_publisher_->publish(polygon_msg);

        RCLCPP_DEBUG(this->get_logger(), "Published scan range polygon.");
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr detected_points_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_publisher_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    bool is_forward_;
    double linear_x_;
    double forward_x_min_, forward_x_max_, forward_y_min_, forward_y_max_;
    double backward_x_min_, backward_x_max_, backward_y_min_, backward_y_max_;
    std::string reference_link_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
