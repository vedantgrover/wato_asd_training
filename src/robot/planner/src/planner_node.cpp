#include "planner_node.hpp"

#include <cmath>
#include <algorithm>
#include <utility>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
   current_map_ = *msg;
   if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
     planPath();
   }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal Reached");
      state_ = State::WAITING_FOR_GOAL;
    } else {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeoeut of progress...");
      planPath();
    }
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return (std::sqrt(dx * dx + dy * dy) < 0.5);
}

void PlannerNode::planPath() {
    if (!goal_received_ || current_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
        return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";

    // Convert start and goal positions to grid coordinates
    CellIndex start = worldToGrid(robot_pose_.position.x, robot_pose_.position.y);
    CellIndex goal = worldToGrid(goal_.point.x, goal_.point.y);

    // A* implementation
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;

    open_set.push(AStarNode(start, heuristic(start, goal)));
    g_score[start] = 0;

    while (!open_set.empty()) {
        CellIndex current = open_set.top().index;
        open_set.pop();

        if (current == goal) {
            path.poses = reconstructPath(came_from, current);
            path_pub_->publish(path);
            return;
        }

        for (const CellIndex& neighbor : getNeighbors(current)) {
            double tentative_g = g_score[current] + 1.0;  // Assuming cost of 1 between adjacent cells

            if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                double f = tentative_g + heuristic(neighbor, goal);
                open_set.push(AStarNode(neighbor, f));
            }
        }
    }

    RCLCPP_WARN(this->get_logger(), "No path found!");
}

CellIndex PlannerNode::worldToGrid(double x, double y) {
  int grid_x = static_cast<int>((x - current_map_.info.origin.position.x) / current_map_.info.resolution);
  int grid_y = static_cast<int>((y - current_map_.info.origin.position.y) / current_map_.info.resolution);
  return CellIndex(grid_x, grid_y);
}

std::pair<double, double> PlannerNode::gridToWorld(int x, int y) {
  double world_x = x * current_map_.info.resolution + current_map_.info.origin.position.x;
  double world_y = y * current_map_.info.resolution + current_map_.info.origin.position.y;
  return std::make_pair(world_x, world_y);
}

bool PlannerNode::isValidCell(const CellIndex& idx) {
  if (idx.x < 0 || idx.x >= current_map_.info.width || idx.y < 0 || idx.y >= current_map_.info.height) {
    return false;
  }

  int cell_index = idx.y * current_map_.info.width + idx.x;
  return current_map_.data[cell_index] < 50;
}

double PlannerNode::heuristic(const CellIndex& a, const CellIndex& b) {
    // Manhattan distance
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

std::vector<CellIndex> PlannerNode::getNeighbors(const CellIndex& current) {
    std::vector<CellIndex> neighbors;
    // 8-connected grid
    const int dx[] = {-1, -1, -1,  0,  0,  1, 1, 1};
    const int dy[] = {-1,  0,  1, -1,  1, -1, 0, 1};

    for (int i = 0; i < 8; ++i) {
        CellIndex neighbor(current.x + dx[i], current.y + dy[i]);
        if (isValidCell(neighbor)) {
            neighbors.push_back(neighbor);
        }
    }
    return neighbors;
}

std::vector<geometry_msgs::msg::PoseStamped> PlannerNode::reconstructPath(
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from,
    const CellIndex& current) {
    
    std::vector<CellIndex> path_indices;
    CellIndex current_cell = current;

    // Reconstruct path backwards
    while (came_from.find(current_cell) != came_from.end()) {
        path_indices.push_back(current_cell);
        current_cell = came_from.at(current_cell);
    }
    path_indices.push_back(current_cell);  // Add start position

    // Reverse path and convert to PoseStamped messages
    std::vector<geometry_msgs::msg::PoseStamped> path;
    for (auto it = path_indices.rbegin(); it != path_indices.rend(); ++it) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->get_clock()->now();

        // Convert grid coordinates to world coordinates
        auto world_coords = gridToWorld(it->x, it->y);
        pose.pose.position.x = world_coords.first;
        pose.pose.position.y = world_coords.second;
        pose.pose.position.z = 0.0;

        // Set orientation (you might want to compute proper orientation based on path direction)
        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;

        path.push_back(pose);
    }

    return path;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
