#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

    void publishMessage();

    void lidarSubscriber(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void costmapPublisher();

    void inflateCostmap();

  private:
    robot::CostmapCore costmap_;

    static const int GRID_SIZE_ = 300;
    const float resolution_ = 0.1;
    const int max_cost_ = 100;
    const float inflation_radius_ = 2.0;  // Increased from 1.5 to give more clearance

    int grid_[GRID_SIZE_][GRID_SIZE_] = {0};

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

//    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;

//    rclcpp::TimerBase::SharedPtr timer_;
};

#endif 