#include "moveit_postprocess/moveit_joint_republisher.hpp"

double deg2rad(double deg) {
    return deg * PI / 180.0;
}

double rad2deg(double rad) {
    return rad * 180.0 / PI;
}

void MoveItJointRepublisher::leftArmCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) {
    auto point = trajectory_msgs::msg::JointTrajectoryPoint();
    point.positions = msg->output.positions;
    point.velocities = msg->output.velocities;
    point.accelerations = msg->output.accelerations;
    point.effort = msg->output.effort;
    point.time_from_start = msg->output.time_from_start;

    left_arm_publisher_->publish(point);
}

void MoveItJointRepublisher::rightArmCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) {
    auto point = trajectory_msgs::msg::JointTrajectoryPoint();
    point.positions = msg->output.positions;
    point.velocities = msg->output.velocities;
    point.accelerations = msg->output.accelerations;
    point.effort = msg->output.effort;
    point.time_from_start = msg->output.time_from_start;

    // Calibration for the right arm
    if (point.positions.size() != 6) {
        RCLCPP_ERROR(this->get_logger(), "Invalid number of joint positions");
        return;
    }
    point.positions[0] = point.positions[0];
    point.positions[1] = deg2rad(rad2deg(point.positions[1]) + 90.0);
    point.positions[2] = deg2rad(rad2deg(point.positions[2]) + 90.0);
    point.positions[3] = -point.positions[3];
    point.positions[4] = deg2rad(-rad2deg(point.positions[4]) + 90.0);
    point.positions[5] = deg2rad(rad2deg(point.positions[5]) + 90.0);

    right_arm_publisher_->publish(point);
}

MoveItJointRepublisher::MoveItJointRepublisher() : Node("moveit_joint_republisher") {
    // Subscribers for controller states
    left_arm_subscriber_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        "/left_arm_controller/controller_state", QOS,
        std::bind(&MoveItJointRepublisher::leftArmCallback, this, std::placeholders::_1));

    right_arm_subscriber_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        "/right_arm_controller/controller_state", QOS,
        std::bind(&MoveItJointRepublisher::rightArmCallback, this, std::placeholders::_1));

    // Publishers for new joint trajectories
    left_arm_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("/left_arm", QOS);
    right_arm_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("/right_arm", QOS);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItJointRepublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
