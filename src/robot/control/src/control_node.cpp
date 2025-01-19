#include "control_node.hpp"
#include <cmath>

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger())) {
    // Initialize subscribers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10,
        [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

    // Initialize publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Create timer for control loop (10Hz)
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::controlLoop() {
    if (!current_path_ || !robot_odom_) {
        return;  // Wait for path and odometry data
    }

    // Find lookahead point
    auto target_pose = findLookaheadDistance();
    if (!target_pose) {
        return;
    }

    // Compute and publish velocity commands
    geometry_msgs::msg::Twist cmd_vel = computeVelocity(*target_pose);
    cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadDistance() {
    if (current_path_->poses.empty()) {
        return std::nullopt;
    }

    auto robot_pose = robot_odom_->pose.pose;
    
    // Find the first point that's at least lookahead_distance_ away
    for (const auto& pose : current_path_->poses) {
        double distance = computeDistance(robot_pose.position, pose.pose.position);
        if (distance >= lookahead_distance_) {
            return pose;
        }
    }

    // If we're close to the final goal, return the last point
    double distance_to_final = computeDistance(
        robot_pose.position,
        current_path_->poses.back().pose.position);
    
    if (distance_to_final < goal_tolerance_) {
        return std::nullopt;  // We've reached the goal
    }

    return current_path_->poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(
    const geometry_msgs::msg::PoseStamped &target) {
    
    geometry_msgs::msg::Twist cmd_vel;
    
    // Calculate heading to target
    double target_yaw = extractYaw(
        robot_odom_->pose.pose.position,
        target.pose.position);
    
    // Extract current robot yaw from quaternion
    double robot_yaw = 2 * std::atan2(
        robot_odom_->pose.pose.orientation.z,
        robot_odom_->pose.pose.orientation.w);
    
    // Calculate angular difference
    double yaw_error = target_yaw - robot_yaw;
    
    // Normalize to [-π, π]
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
    
    // Set linear and angular velocities
    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = 2.0 * yaw_error;  // Simple P controller for angular velocity
    
    return cmd_vel;
}

double ControlNode::computeDistance(
    const geometry_msgs::msg::Point &a,
    const geometry_msgs::msg::Point &b) {
    
    return std::sqrt(
        std::pow(b.x - a.x, 2) +
        std::pow(b.y - a.y, 2));
}

double ControlNode::extractYaw(
    const geometry_msgs::msg::Point &a,
    const geometry_msgs::msg::Point &b) {
    
    return std::atan2(b.y - a.y, b.x - a.x);
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
