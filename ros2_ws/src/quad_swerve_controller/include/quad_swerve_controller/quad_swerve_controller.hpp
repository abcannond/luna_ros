#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <memory>
#include "motor_interface.hpp"

class QuadSwerveController : public rclcpp::Node
{
public:
    QuadSwerveController();

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void updateMotorCommands();

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    std::vector<std::shared_ptr<MotorInterface>> wheels_;

    // store last received cmd_vel
    double linear_x_ = 0.0;
    double linear_y_ = 0.0;
    double angular_z_ = 0.0;

    double wheel_track_ = 0.3;  // meters, adjustable
    double wheel_base_ = 0.4;   // meters, adjustable
    double wheel_radius_ = 0.05; // meters, adjustable
};
