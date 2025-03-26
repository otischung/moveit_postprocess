#pragma once
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "moveit_postprocess/params.hpp"

static inline double deg2rad(double deg);
static inline double rad2deg(double rad);

class MoveItJointRepublisher : public rclcpp::Node {
   private:
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr left_arm_subscriber_;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr right_arm_subscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr left_arm_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr right_arm_publisher_;

    void leftArmCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
    void rightArmCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);

   public:
    MoveItJointRepublisher();
};
