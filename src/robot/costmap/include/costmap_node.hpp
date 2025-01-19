#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

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
    const float inflation_radius_ = 1.6;

    int grid_[GRID_SIZE_][GRID_SIZE_] = {0};

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
//    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;

//    rclcpp::TimerBase::SharedPtr timer_;
};

#endif 