#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Initialize publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Initialize timer
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

  global_map_.header.stamp = this->now();
  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = resolution_;
  global_map_.info.width = GRID_SIZE_;
  global_map_.info.height = GRID_SIZE_;
  global_map_.info.origin.position.x = -1 * (GRID_SIZE_ * resolution_) / 2.0;  // Center the map on the field
  global_map_.info.origin.position.y = -1 * (GRID_SIZE_ * resolution_) / 2.0;  // Center the map on the field
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.resize(GRID_SIZE_ * GRID_SIZE_, 0);
  for (int i = 0; i < 300 * 300; i++) {
    global_map_.data[i] = 0;
  }
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
  global_map_.info = costmap->info;
  latest_costmap_ = *costmap;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // Compute distance traveled
  double distance = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));
  if (distance >= distance_threshold_meters_) {
    last_x_ = x;
    last_y_ = y;
    should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_updated_) {
    RCLCPP_INFO(this->get_logger(), "Updating map");
    for (int i = 0; i < 300 * 300; i++) {
      for (int j = 0; j < 300 * 300; j++) {
        global_map_.data[i] = 0;
      }
    }
    integrateCostMap();
    map_pub_->publish(global_map_);
    should_update_map_ = false;
    costmap_updated_ = false;
  }
}

//void MapMemoryNode::integrateCostMap() {
//  if (latest_costmap_.info.width == 0 || latest_costmap_.info.height == 0) {
//    return;
//  }
//
//  // Calculate offset between global map and costmap
//  double offset_x = (latest_costmap_.info.origin.position.x - global_map_.info.origin.position.x) / global_map_.info.resolution;
//  double offset_y = (latest_costmap_.info.origin.position.y - global_map_.info.origin.position.y) / global_map_.info.resolution;
//
//  for (unsigned int y = 0; y < latest_costmap_.info.height; y++) {
//    for (unsigned int x = 0; x < latest_costmap_.info.width; x++) {
//      // Calculate global map coordinates
//      int global_x = static_cast<int>(x + offset_x);
//      int global_y = static_cast<int>(y + offset_y);
//
//      // Check if the point is within global map bounds
//      if (global_x >= 0 && global_x < static_cast<int>(global_map_.info.width) &&
//          global_y >= 0 && global_y < static_cast<int>(global_map_.info.height)) {
//
//        unsigned int costmap_index = y * latest_costmap_.info.width + x;
//        unsigned int global_index = global_y * global_map_.info.width + global_x;
//
//        if (global_map_.data[global_index] == 0 && latest_costmap_.data[costmap_index] > 0) {
//          global_map_.data[global_index] = latest_costmap_.data[costmap_index];
//        } else if (global_map_.data[global_index] < latest_costmap_.data[costmap_index] &&
//                 latest_costmap_.data[costmap_index] > 50) {
//          global_map_.data[global_index] = latest_costmap_.data[costmap_index];
//                 }
//        global_map_.data[global_index] *= 0.92;
//          }
//    }
//  }
//}

void MapMemoryNode::integrateCostMap() {
    if (latest_costmap_.info.width == 0 || latest_costmap_.info.height == 0) {
        return;
    }

    for (int i = 0; i < GRID_SIZE_; i++) {
        for (int j = 0; j < GRID_SIZE_; j++) {
            int index = GRID_SIZE_ * i + j;
            
            if (global_map_.data[index] == 0 && latest_costmap_.data[index] > 0) {
                global_map_.data[index] = latest_costmap_.data[index];
            } else if (global_map_.data[index] < latest_costmap_.data[index] && 
                      latest_costmap_.data[index] > 50) {
                global_map_.data[index] = latest_costmap_.data[index];
            }
            
            // Apply decay factor
            global_map_.data[index] *= 0.92;
        }
    }
}

// void MapMemoryNode::integrateCostMap() {
//   if (latest_costmap_.info.width == 0 || latest_costmap_.info.height == 0) {
//     return;
//   }

//   for (int i = 0; i < GRID_SIZE_; i++) {
//     for (int j = 0; j < GRID_SIZE_; j++) {
//       if (global_map_.data[GRID_SIZE_ * i + j] == 0 && latest_costmap_.data[GRID_SIZE_ * i + j] > 0) {
//         global_map_.data[GRID_SIZE_ * i + j] = latest_costmap_.data[GRID_SIZE_*i + j];
//       } else if (global_map_.data[GRID_SIZE_ * i + j] < latest_costmap_.data[GRID_SIZE_*i + j]  && latest_costmap_.data[GRID_SIZE_ * i + j] > 50) {
//         global_map_.data[GRID_SIZE_ * i + j] = latest_costmap_.data[GRID_SIZE_*i + j];
//       }
//       global_map_.data[GRID_SIZE_ * i + j] *= 0.92;
//     }
//   }
// }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
