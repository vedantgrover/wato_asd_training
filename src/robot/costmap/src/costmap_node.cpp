#include <chrono>
#include <memory>

#include "costmap_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::lidarSubscriber, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered",  // or whatever the odometry topic is
        10,
        std::bind(&CostmapNode::odomCallback, this, std::placeholders::_1)
    );
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
                int cost = static_cast<int>(max_cost_ * (1.0 - distance / inflation_radius_));
                if (cost > 10) {
                  grid_[new_x][new_y] = std::max(grid_[new_x][new_y], cost);
                }
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

void CostmapNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Update robot's position
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    // Convert quaternion to yaw (theta)
    // You might need to add tf2 headers for this
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    robot_theta_ = yaw;

    RCLCPP_DEBUG(this->get_logger(), 
        "Robot pose updated - x: %f, y: %f, theta: %f", 
        robot_x_, robot_y_, robot_theta_
    );
}

void CostmapNode::lidarSubscriber(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Clear the grid
  for (int i = 0; i < GRID_SIZE_; i++) {
    for (int j = 0; j < GRID_SIZE_; j++) {
      grid_[i][j] = 0;
    }
  }

  // Get robot's position in the field
  // You'll need to get this from tf or odometry
  float robot_x = robot_x_;  // robot's x position in field coordinates
  float robot_y = robot_y_;  // robot's y position in field coordinates
  float robot_theta = robot_theta_;  // robot's orientation in field coordinates

  float angleMinRadians = scan->angle_min;
  float angleIncrementRadians = scan->angle_increment;
  auto ranges = scan->ranges;

  for (size_t i = 0; i < ranges.size(); i++) {
    float angleRadians = angleMinRadians + (i * angleIncrementRadians);
    float range = ranges[i];

    float map_center_meters = (GRID_SIZE_ * resolution_) / 2.0;

    if (range < scan->range_max && range > scan->range_min) {
      // Convert laser scan to robot frame
      float x_robot = range * cos(angleRadians);
      float y_robot = range * sin(angleRadians);

      // Transform from robot frame to field frame
      float x_field = robot_x + (x_robot * cos(robot_theta) - y_robot * sin(robot_theta));
      float y_field = robot_y + (x_robot * sin(robot_theta) + y_robot * cos(robot_theta));

      // Convert to grid coordinates
      int x = static_cast<int>((x_field + map_center_meters) / resolution_);
      int y = static_cast<int>((y_field + map_center_meters) / resolution_);

      if (x >= 0 && x < GRID_SIZE_ && y >= 0 && y < GRID_SIZE_) {
        grid_[x][y] = max_cost_;
      }
    }
  }

  inflateCostmap();

  auto msg = nav_msgs::msg::OccupancyGrid();
  msg.header = scan->header;
  msg.header.frame_id = "sim_world";
  msg.info.width = GRID_SIZE_;
  msg.info.height = GRID_SIZE_;
  msg.info.resolution = resolution_;

  msg.info.origin.position.x = -1*(GRID_SIZE_ * resolution_) / 2.0;  // Set to field origin
  msg.info.origin.position.y = -1*(GRID_SIZE_ * resolution_) / 2.0;  // Set to field origin
  msg.info.origin.orientation.w = 1.0;

  msg.data.resize(GRID_SIZE_ * GRID_SIZE_);
  for (int i = 0; i < GRID_SIZE_; i++) {
    for (int j = 0; j < GRID_SIZE_; j++) {
      msg.data[j * GRID_SIZE_ + i] = grid_[i][j];
    }
  }
  costmap_pub_->publish(msg);
}

// void CostmapNode::lidarSubscriber(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
//   for (int i = 0; i < GRID_SIZE_; i++) {
//     for (int j = 0; j < GRID_SIZE_; j++) {
//       grid_[i][j] = 0;
//     }
//   }

//   float angleMinRadians = scan->angle_min;
//   float angleMaxRadians = scan->angle_max;
//   float angleIncrementRadians = scan->angle_increment;
//   auto ranges = scan->ranges;

//   const int grid_center = GRID_SIZE_ / 2;

//   for (size_t i = 0; i < ranges.size(); i++) {
//     float angleRadians = angleMinRadians + (i * angleIncrementRadians);
//     float range = ranges[i];

//     if (range < scan->range_max && range > scan->range_min) {
//       float x_real = range * cos(angleRadians);
//       float y_real = range * sin(angleRadians);

//       int x = static_cast<int>((x_real / resolution_) + grid_center);
//       int y = static_cast<int>((y_real / resolution_) + grid_center);

//       if (x >= 0 && y >= 0 && x < GRID_SIZE_ && y < GRID_SIZE_) {
//         grid_[x][y] = max_cost_;
//       }
//     }
//   }

//   inflateCostmap();

//   auto msg = nav_msgs::msg::OccupancyGrid();
//   msg.header = scan->header;
//   msg.header.frame_id = "map";
//   msg.info.width = GRID_SIZE_;
//   msg.info.height = GRID_SIZE_;
//   msg.info.resolution = resolution_;
//   msg.info.origin.position.x = -1 * (GRID_SIZE_ / 2) * resolution_;
//   msg.info.origin.position.y = -1 * (GRID_SIZE_ / 2) * resolution_;

//   msg.data.resize(300 * 300);
//   for (int i = 0; i < 300; i++) {
//     for (int j = 0; j < 300; j++) {
//       msg.data[j * 300 + i] = grid_[i][j];
//     }
//   }
//   costmap_pub_->publish(msg);
// }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}