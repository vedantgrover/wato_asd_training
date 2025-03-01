#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <queue>
#include <unordered_map>

#include "planner_core.hpp"
#include "planner_types.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

  private:
    robot::PlannerCore planner_;

    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;

    bool goal_received_ = false;

    bool goalReached();
    void planPath();

    CellIndex worldToGrid(double x, double y);
    std::pair<double, double> gridToWorld(int x, int y);
    bool isValidCell(const CellIndex& idx);
    double heuristic(const CellIndex& a, const CellIndex& b);
    std::vector<CellIndex> getNeighbors(const CellIndex& current);
    std::vector<geometry_msgs::msg::PoseStamped> reconstructPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from, const CellIndex& current);
    std::vector<geometry_msgs::msg::PoseStamped> smoothPath(const std::vector<geometry_msgs::msg::PoseStamped>& original_path);
};

#endif 
