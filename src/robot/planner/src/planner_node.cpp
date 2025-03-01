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
  double distance = std::sqrt(dx * dx + dy * dy);
  
  // Get robot's current orientation
  tf2::Quaternion q(
    robot_pose_.orientation.x,
    robot_pose_.orientation.y,
    robot_pose_.orientation.z,
    robot_pose_.orientation.w
  );
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  
  // Calculate angle to goal
  double angle_to_goal = std::atan2(dy, dx);
  double angle_diff = std::abs(angle_to_goal - yaw);
  
  // Normalize angle difference to [0, π]
  while (angle_diff > M_PI) {
    angle_diff = std::abs(angle_diff - 2 * M_PI);
  }
  
  // Consider goal reached if robot is close enough and facing roughly the right direction
  return (distance < 0.3 && angle_diff < M_PI/4);
}

void PlannerNode::planPath() {
    if (!goal_received_ || current_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
        return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "sim_world";

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
            // Get cell cost from map
            int cell_index = neighbor.y * current_map_.info.width + neighbor.x;
            double obstacle_cost = current_map_.data[cell_index] / 20.0;  // More aggressive obstacle avoidance
            
            // Higher cost near obstacles
            double tentative_g = g_score[current] + 1.0 + obstacle_cost * 2.0;

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
  // Check bounds
  if (idx.x < 0 || static_cast<uint32_t>(idx.x) >= current_map_.info.width ||
      idx.y < 0 || static_cast<uint32_t>(idx.y) >= current_map_.info.height) {
    return false;
  }

  // Get cell cost
  int cell_index = idx.y * current_map_.info.width + idx.x;
  int cell_cost = current_map_.data[cell_index];

  // Direct obstacle check
  if (cell_cost >= 90) {  // Definitely an obstacle
    return false;
  }

  // Check for high-cost areas
  if (cell_cost >= 50) {  // Likely obstacle or very close to one
    return false;
  }

  // For path planning, we want to be more permissive to allow finding paths
  // but still avoid dangerous areas
  const int safety_radius = 1;  // Reduced from 2 to allow more path options
  int high_cost_count = 0;

  for (int dx = -safety_radius; dx <= safety_radius; dx++) {
    for (int dy = -safety_radius; dy <= safety_radius; dy++) {
      int nx = idx.x + dx;
      int ny = idx.y + dy;
      
      // Skip if out of bounds
      if (nx < 0 || static_cast<uint32_t>(nx) >= current_map_.info.width ||
          ny < 0 || static_cast<uint32_t>(ny) >= current_map_.info.height) {
        continue;
      }

      int neighbor_index = ny * current_map_.info.width + nx;
      if (current_map_.data[neighbor_index] >= 75) {  // Count cells very close to obstacles
        high_cost_count++;
      }
    }
  }

  // If too many nearby cells are high-cost, consider this cell invalid
  return high_cost_count <= 2;  // Allow some high-cost cells but not too many
}

double PlannerNode::heuristic(const CellIndex& a, const CellIndex& b) {
    // Base distance using Euclidean distance
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double base_cost = std::sqrt(dx * dx + dy * dy);

    // Add cost based on cell occupancy to encourage paths through clear areas
    int cell_index = a.y * current_map_.info.width + a.x;
    if (cell_index >= 0 && static_cast<size_t>(cell_index) < current_map_.data.size()) {
        double occupancy_cost = current_map_.data[cell_index] / 25.0;  // Normalize to 0-4 range
        return base_cost * (1.0 + occupancy_cost);  // Increase cost near obstacles
    }
    
    return base_cost;
}

std::vector<geometry_msgs::msg::PoseStamped> PlannerNode::smoothPath(
    const std::vector<geometry_msgs::msg::PoseStamped>& original_path) {
    if (original_path.size() < 3) return original_path;

    std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;
    smoothed_path.push_back(original_path.front());

    // Bezier curve smoothing
    for (size_t i = 1; i < original_path.size() - 1; i++) {
        const auto& prev = original_path[i-1].pose.position;
        const auto& curr = original_path[i].pose.position;
        const auto& next = original_path[i+1].pose.position;

        // Create more intermediate points for smoother curves
        int num_points = 8;  // Increased from 5 to 8 for smoother curves
        for (int j = 0; j < num_points; j++) {
            double t = static_cast<double>(j) / num_points;
            double mt = 1.0 - t;
            
            geometry_msgs::msg::PoseStamped pose;
            pose.header = original_path[i].header;
            
            // Quadratic Bezier curve
            pose.pose.position.x = mt*mt*prev.x + 2*mt*t*curr.x + t*t*next.x;
            pose.pose.position.y = mt*mt*prev.y + 2*mt*t*curr.y + t*t*next.y;
            pose.pose.position.z = 0.0;

            // Calculate orientation based on path direction
            double path_dx = 2*mt*(-prev.x + curr.x) + 2*t*(curr.x - next.x);
            double path_dy = 2*mt*(-prev.y + curr.y) + 2*t*(curr.y - next.y);
            double yaw = std::atan2(path_dy, path_dx);

            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();

            // Check if the interpolated point and its surroundings are safe
            CellIndex point = worldToGrid(pose.pose.position.x, pose.pose.position.y);
            
            // Skip this interpolated point if it's too close to obstacles
            if (!isValidCell(point)) {
                continue;
            }

            // Check points between current and previous point for safety
            if (!smoothed_path.empty()) {
                const auto& prev_pose = smoothed_path.back();
                bool path_safe = true;
                
                // Check several points along the line between prev and current
                for (int k = 1; k <= 3; k++) {
                    double ratio = static_cast<double>(k) / 4.0;
                    double check_x = prev_pose.pose.position.x * (1 - ratio) + pose.pose.position.x * ratio;
                    double check_y = prev_pose.pose.position.y * (1 - ratio) + pose.pose.position.y * ratio;
                    
                    CellIndex check_point = worldToGrid(check_x, check_y);
                    if (!isValidCell(check_point)) {
                        path_safe = false;
                        break;
                    }
                }
                
                if (!path_safe) {
                    continue;
                }
            }

            smoothed_path.push_back(pose);
        }
    }

    smoothed_path.push_back(original_path.back());
    return smoothed_path;
}

std::vector<CellIndex> PlannerNode::getNeighbors(const CellIndex& current) {
    std::vector<CellIndex> neighbors;
    
    // Define movements: straight first, then diagonals
    const int dx[] = {0, 1, 0, -1, 1, 1, -1, -1};
    const int dy[] = {1, 0, -1, 0, 1, -1, -1, 1};
    const bool is_diagonal[] = {false, false, false, false, true, true, true, true};

    for (int i = 0; i < 8; ++i) {
        CellIndex neighbor(current.x + dx[i], current.y + dy[i]);
        
        // Skip if neighbor cell is invalid
        if (!isValidCell(neighbor)) {
            continue;
        }

        // For diagonal movements, check both adjacent cells
        if (is_diagonal[i]) {
            CellIndex corner1(current.x + dx[i], current.y);
            CellIndex corner2(current.x, current.y + dy[i]);
            
            // Skip diagonal if either adjacent cell is invalid
            if (!isValidCell(corner1) || !isValidCell(corner2)) {
                continue;
            }

            // Get costs of corner cells
            int corner1_idx = corner1.y * current_map_.info.width + corner1.x;
            int corner2_idx = corner2.y * current_map_.info.width + corner2.x;
            
            // Skip diagonal if either corner has high cost
            if (current_map_.data[corner1_idx] > 40 || current_map_.data[corner2_idx] > 40) {
                continue;
            }
        }

        neighbors.push_back(neighbor);
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

    // Reverse path and convert to PoseStamped messages with proper orientations
    std::vector<geometry_msgs::msg::PoseStamped> raw_path;
    for (auto it = path_indices.rbegin(); it != path_indices.rend(); ++it) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "sim_world";
        pose.header.stamp = this->get_clock()->now();

        // Convert grid coordinates to world coordinates
        auto world_coords = gridToWorld(it->x, it->y);
        pose.pose.position.x = world_coords.first;
        pose.pose.position.y = world_coords.second;
        pose.pose.position.z = 0.0;

        // Calculate orientation based on path direction
        if (it != path_indices.rbegin() && std::next(it) != path_indices.rend()) {
            // For middle points, use previous and next points to determine orientation
            auto prev_coords = gridToWorld(std::prev(it)->x, std::prev(it)->y);
            auto next_coords = gridToWorld(std::next(it)->x, std::next(it)->y);
            
            double dx = next_coords.first - prev_coords.first;
            double dy = next_coords.second - prev_coords.second;
            double yaw = std::atan2(dy, dx);

            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
        } else {
            // For start/end points, use adjacent point
            auto adjacent = (it == path_indices.rbegin()) ?
                          gridToWorld(std::next(it)->x, std::next(it)->y) :
                          gridToWorld(std::prev(it)->x, std::prev(it)->y);
            
            double dx = adjacent.first - world_coords.first;
            double dy = adjacent.second - world_coords.second;
            double yaw = std::atan2(dy, dx);

            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
        }

        raw_path.push_back(pose);
    }

    // Apply path smoothing
    return smoothPath(raw_path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
