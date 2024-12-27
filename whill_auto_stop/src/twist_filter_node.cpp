// Copyright (c) 2024 WHILL, Inc.
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

class TwistFilterNode : public rclcpp::Node {
public:
    TwistFilterNode() : Node("twist_filter_node"), enable_publish_(false), negative_logic_(false) {
        // Declare and get negative_logic parameter
        this->declare_parameter<bool>("negative_logic", false);
        this->get_parameter("negative_logic", negative_logic_);

        // Subscriber for Twist messages
        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "input_twist", 10,
            std::bind(&TwistFilterNode::twistCallback, this, std::placeholders::_1));

        // Subscriber for bool messages
        bool_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "enable", 10,
            std::bind(&TwistFilterNode::boolCallback, this, std::placeholders::_1));

        // Publisher for filtered Twist messages
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("output_twist", 10);

        RCLCPP_INFO(this->get_logger(), "Twist Filter Node has been started.");
        RCLCPP_INFO(this->get_logger(), "Negative logic is %s", negative_logic_ ? "enabled" : "disabled");
    }

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if ((enable_publish_ && !negative_logic_) || (!enable_publish_ && negative_logic_)) {
            twist_publisher_->publish(*msg);
            RCLCPP_INFO(this->get_logger(), "Publishing Twist message.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Publishing disabled, Twist message discarded.");
        }
    }

    void boolCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        enable_publish_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Enable flag set to: %s", negative_logic_ ? (enable_publish_ ? "False" : "True") : (enable_publish_ ? "True" : "False"));
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

    bool enable_publish_;
    bool negative_logic_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistFilterNode>());
    rclcpp::shutdown();
    return 0;
}
