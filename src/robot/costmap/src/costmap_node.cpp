#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::lidarSubscriber, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

//  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
//  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CostmapNode::publishMessage, this));
}

//void CostmapNode::publishMessage() {
//  auto message = std_msgs::msg::String();
//  message.data = "Hello ROS2!";
//  RCLCPP_INFO(this->get_logger(), "publishing %s", message.data.c_str());
//  string_pub_->publish(message);
//}
void CostmapNode::inflateCostmap() {
  int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);

  for (int x = 0; x < GRID_SIZE_; x++) {
    for (int y = 0; y < GRID_SIZE_; y++) {
      if (grid_[x][y] == 100) {  // If cell is an obstacle
        // Inflate around obstacle
        for (int dx = -inflation_cells; dx <= inflation_cells; dx++) {
          for (int dy = -inflation_cells; dy <= inflation_cells; dy++) {
            int new_x = x + dx;
            int new_y = y + dy;

            if (new_x >= 0 && new_x < GRID_SIZE_ && new_y >= 0 && new_y < GRID_SIZE_) {
              double distance = std::sqrt(dx * dx + dy * dy) * resolution_;

              if (distance <= inflation_radius_) {
                int cost = static_cast<int>(max_cost_ * (1.0 - distance/inflation_radius_));

                grid_[new_x][new_y] = std::max(grid_[new_x][new_y], cost);
              }
            }
          }
        }
      }
    }
  }
}

//void CostmapNode::inflateCostmap() {
//  int inflation_radius_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
//
//  std::vector<std::vector<float>> distance_kernel(2 * inflation_radius_cells + 1, std::vector<float>(2 * inflation_radius_cells + 1, 0));
//
//  for (int dx = -inflation_radius_cells; dx <= inflation_radius_cells; dx++) {
//    for (int dy = -inflation_radius_cells; dy <= inflation_radius_cells; dy++) {
//      float distance = std::sqrt(dx * dx + dy * dy);
//      if (distance <= inflation_radius_cells) {
//        distance_kernel[dx + inflation_radius_cells][dy + inflation_radius_cells] = 1.0f - (distance / inflation_radius_cells);
//      }
//    }
//  }
//
//  int inflated_map[GRID_SIZE_][GRID_SIZE_] = {0};
//
//  for (int x = 0; x < GRID_SIZE_; ++x) {
//    for (int y = 0; y < GRID_SIZE_; ++y) {
//      if (grid_[x][y] == max_cost_) {  // Occupied cell
//        for (int dx = -inflation_radius_cells; dx <= inflation_radius_cells; ++dx) {
//          for (int dy = -inflation_radius_cells; dy <= inflation_radius_cells; ++dy) {
//            int nx = x + dx;
//            int ny = y + dy;
//
//            // Check bounds
//            if (nx >= 0 && nx < GRID_SIZE_ && ny >= 0 && ny < GRID_SIZE_) {
//              // Apply the inflation kernel
//              float cost = max_cost_ * distance_kernel[dx + inflation_radius_cells][dy + inflation_radius_cells];
//              inflated_map[nx][ny] = std::max(inflated_map[nx][ny], static_cast<int>(cost));
//            }
//          }
//        }
//      }
//    }
//  }
//
//  // Copy the inflated map back to the original grid
//  for (int x = 0; x < GRID_SIZE_; ++x) {
//    for (int y = 0; y < GRID_SIZE_; ++y) {
//      grid_[x][y] = std::max(grid_[x][y], inflated_map[x][y]);
//    }
//  }
//}

void CostmapNode::lidarSubscriber(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  float angleMinRadians = scan->angle_min;
  float angleMaxRadians = scan->angle_max;
  float angleIncrementRadians = scan->angle_increment;
  auto ranges = scan->ranges;

  const int grid_center = GRID_SIZE_ / 2;

  for (size_t i = 0; i < ranges.size(); i++) {
    float angleRadians = angleMinRadians + (i * angleIncrementRadians);
    float range = ranges[i];

    if (range < scan->range_max && range > scan->range_min) {
      float x_real = range * cos(angleRadians);
      float y_real = range * sin(angleRadians);

      int x = static_cast<int>((x_real / resolution_) + grid_center);
      int y = static_cast<int>((y_real / resolution_) + grid_center);

      if (x >= 0 && y >= 0 && x < GRID_SIZE_ && y < GRID_SIZE_) {
        grid_[x][y] = max_cost_;
      }
    }
  }

  inflateCostmap();

  auto msg = nav_msgs::msg::OccupancyGrid();
  msg.header = scan->header;
  msg.info.width = 300;
  msg.info.height = 300;
  msg.info.resolution = resolution_;
  msg.info.origin.position.x = -15;
  msg.info.origin.position.y = -15;

  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data.resize(300 * 300);
  for (int i = 0; i < 300; i++) {
    for (int j = 0; j < 300; j++) {
      msg.data[i * 300 + j] = grid_[i][j];
    }
  }
  costmap_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}