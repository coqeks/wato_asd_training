#include <chrono>
#include <memory>
#include <array>
#include <vector>
 
#include "costmap_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
 
// Initializer list (what's after :), Initializes Node() base class and costmap_ property before constructor body runs.
// Convention is not to put subscription / publisher on those, but cheap / necessary member variables
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10); 
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
  // this = pointer to the object a function was called on
  laser_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::processLaserMsg, this, std::placeholders::_1)); // Because readLaserMsg takes one argument, you need a placeholder 
  // 
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

std::vector<std::vector<int>> CostmapNode::initGrid() {
  std::vector<std::vector<int>> grid(this->grid_width / this->grid_res, std::vector<int>(grid_height / this->grid_res, 0));
  return grid;
}

// LaserScan:
//  - angle_min and angle_max, with angle_increment
//  - ranges (up to 256), distances ordered by angle (starting from min to max with increment of angle_increment)
//  - See how many coordinates are in a grid, if none = zero chance of danger, if > 0 than increased chance
//  - Discard and range in ranges, simple conditional check
//  - Process:
//    - Calculate x and y coordinate of each range items -> find the cell corresponding to the coordinates, mark as occupied (0 -> 100) -> return the o_grid
//    - Grid increment = 0.1m
//    - Grid size = 10m x 10m 
void CostmapNode::processLaserMsg(sensor_msgs::msg::LaserScan::SharedPtr laserScan) {

  std::vector<std::vector<int>> o_grid = initGrid();
  float min_angle = laserScan->angle_min;

  for (int i = 0; i < laserScan->ranges.size(); i++) {

    // Make sure distance is not too small / big
    if (laserScan->ranges[i] > laserScan->range_max || laserScan->ranges[i] < laserScan->range_min) {
      continue;
    }

    float angle = min_angle + laserScan->angle_increment * i;
    float x_dis = laserScan->ranges[i] * cos(angle);
    float y_dis = laserScan->ranges[i] * sin(angle);
    float grid_res = this->grid_res;
    // static_cast to ensure what we get is an int
    // displacement / grid resolution gives me displacement with respect to the grid's scale, 
    // which i can add to the centre of axis to find the position in grid.
    int x_grid = static_cast<int>(x_dis / grid_res) + grid_width / 2; 
    int y_grid = static_cast<int>(y_dis / grid_res) + grid_height / 2;

    // Make sure grid values are within the grid
    if (x_grid < grid_width || x_grid >= 0 || y_grid < grid_height || y_grid >= 0) {
      o_grid[y_grid][x_grid] = 100; // grid certainly has obstacle within
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>()); // Blocks the thread to keep the node (Costmap) running, starts an event loop for subscription / timer 
  rclcpp::shutdown();
  return 0;
}