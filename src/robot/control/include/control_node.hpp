#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <optional>
#include <algorithm>
#include <cmath>

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

    void controlLoop();
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadDistance();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target);

    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);

    double extractYaw(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);

  private:
    robot::ControlCore control_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Data
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;

    double lookahead_distance_ = 1.0;
    double goal_tolerance_ = 0.1;
    double linear_speed_ = 2.5;
};

#endif
