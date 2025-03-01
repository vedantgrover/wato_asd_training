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
    
    // Calculate current robot speed
    double current_speed = std::sqrt(
        std::pow(robot_odom_->twist.twist.linear.x, 2) +
        std::pow(robot_odom_->twist.twist.linear.y, 2)
    );
    
    // Adjust lookahead distance based on speed
    double dynamic_lookahead = std::max(
        0.5,  // minimum lookahead
        std::min(2.0,  // maximum lookahead
                0.5 + current_speed * 0.5)  // speed-based scaling
    );

    // Find closest point on path
    size_t closest_idx = 0;
    double min_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < current_path_->poses.size(); ++i) {
        double distance = computeDistance(robot_pose.position, current_path_->poses[i].pose.position);
        if (distance < min_distance) {
            min_distance = distance;
            closest_idx = i;
        }
    }

    // Look ahead from closest point
    for (size_t i = closest_idx; i < current_path_->poses.size(); ++i) {
        double distance = computeDistance(robot_pose.position, current_path_->poses[i].pose.position);
        if (distance >= dynamic_lookahead) {
            return current_path_->poses[i];
        }
    }

    // If we're close to the final goal, check if we should stop
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
    tf2::Quaternion q(
        robot_odom_->pose.pose.orientation.x,
        robot_odom_->pose.pose.orientation.y,
        robot_odom_->pose.pose.orientation.z,
        robot_odom_->pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, robot_yaw;
    m.getRPY(roll, pitch, robot_yaw);
    
    // Calculate angular difference
    double yaw_error = target_yaw - robot_yaw;
    
    // Normalize to [-π, π]
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
    
    // Calculate distance to target
    double distance = computeDistance(robot_odom_->pose.pose.position, target.pose.position);

    // Check if we're at the final goal point
    bool is_final_point = (target.pose.position.x == current_path_->poses.back().pose.position.x &&
                          target.pose.position.y == current_path_->poses.back().pose.position.y);

    // If we're very close to the goal and it's the final point, stop completely
    if (is_final_point && distance < goal_tolerance_ * 1.5) {
        // If we're well-aligned, stop completely
        if (std::abs(yaw_error) < 0.1) {  // About 5.7 degrees
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            return cmd_vel;
        }
        // If not well-aligned, only rotate (no linear motion)
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = std::clamp(yaw_error, -0.5, 0.5);  // Gentle rotation
        return cmd_vel;
    }

    // Normal path following behavior
    double angular_factor = std::cos(yaw_error);  // Reduce speed when turning
    double distance_factor = std::min(distance / lookahead_distance_, 1.0);  // Slow down when approaching target
    
    // Compute velocities with improved control
    cmd_vel.linear.x = linear_speed_ * angular_factor * distance_factor;
    
    // PD controller for angular velocity with dead zone
    static double prev_error = 0.0;
    double error_diff = (yaw_error - prev_error) / 0.1;  // dt = 0.1s (control loop rate)
    
    // Add dead zone for small angular errors
    if (std::abs(yaw_error) < 0.05) {  // About 2.8 degrees
        cmd_vel.angular.z = 0.0;
    } else {
        cmd_vel.angular.z = 1.5 * yaw_error + 0.5 * error_diff;  // kp = 1.5, kd = 0.5
        // Limit maximum angular velocity
        cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -1.5, 1.5);
    }
    prev_error = yaw_error;
    
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
