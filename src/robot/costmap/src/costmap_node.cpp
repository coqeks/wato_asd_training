#include <chrono>
#include <memory>
#include <array>
#include <vector>
#include <cmath>
 
#include "costmap_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
 
// Initializer list (what's after :), Initializes Node() base class and costmap_ property before constructor body runs.
// Convention is not to put subscription / publisher on those, but cheap / necessary member variables
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  // this = pointer to the object a function was called on
  laser_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::processLaserMsg, this, std::placeholders::_1)); // Because readLaserMsg takes one argument, you need a placeholder 
}
 
// // Define the timer to publish a message every 500ms
// void CostmapNode::publishMessage() {
//   auto message = std_msgs::msg::String();
//   message.data = "Hello, ROS 2!";
//   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//   string_pub_->publish(message);
// }

void CostmapNode::publishCostmapMsg() {
  auto message = nav_msgs::msg::OccupancyGrid();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "costmap";  

  message.info.resolution = this->grid_res;
  message.info.width = this->width_cells;
  message.info.height = this->height_cells;
  message.info.origin.position.x = -(this->grid_width / 2.0);
  message.info.origin.position.y = -(this->grid_height / 2.0);

  message.data.resize(message.info.width * message.info.height);
  for (size_t y = 0; y < this->o_grid.size(); y++) {
    for (size_t x = 0; x < this->o_grid[y].size(); x++) {
      message.data[y * message.info.width + x] = static_cast<int8_t>(this->o_grid[y][x]);
    }
  }

  costmap_pub_->publish(message);
}


std::vector<std::vector<int>> CostmapNode::initGrid() {
  std::vector<std::vector<int>> grid(this->height_cells, std::vector<int>(this->width_cells, -1));
  return grid;
}

// If a cell got marked by a laser, cells between the marked cell and origin must be 'safe'
// Mark those cells as 0, as they are known to be safe
// x0, y0 = position of the origin
// x1, y1 = position of the marked cell
void CostmapNode::markFreeLine(int y0, int x0, int y1, int x1) {
  int dx = std::abs(x1 - x0); // Get displacements
  int dy = -std::abs(y1 - y0); 
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  int x = x0;
  int y = y0;

  while (true) {
    // Loop until we traced all the way up to marked cell
    if (x == x1 && y == y1) {
      break;
    }
    if (x >= 0 && x < width_cells && y >= 0 && y < height_cells) {
      if (this->o_grid[y][x] != 100) {
        this->o_grid[y][x] = 0;
      }
    }
    int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
  }
}

void CostmapNode::inflateGridCell(int cell_y, int cell_x, int min, int max) {
  int box_size = static_cast<int>(this->inflation_r / this->grid_res);
  // For every row (y level), go through every column (cells)
  for (int box_y = cell_y - box_size; box_y <= cell_y + box_size; box_y++) {
    // continue if row isn't outside o-grid
    if (box_y < min || box_y > max) {
      continue;
    }
    for (int box_x = cell_x - box_size; box_x <= cell_x + box_size; box_x++) {
      if (box_x < min || box_x > max) {
        continue;
      }

      int x_distance = box_x - cell_x; // Get distance (in cells)
      int y_distance = box_y - cell_y;
      double distance = std::sqrt(x_distance*x_distance + y_distance*y_distance) * this->grid_res; // Get euclidean distance in meters (bc inflation_r is in meters)
      double inflated_cost = this->max_cost * (1.0 - (distance / this->inflation_r)); // Calculate inflated cost 

      if (distance > this->inflation_r) { // If distance is larger than radius, skip
        continue;
      }
      if (box_y < 0 || box_y >= static_cast<int>(o_grid.size()) || box_x < 0 || box_x >= static_cast<int>(o_grid[0].size())) { // Condition check for OOB cases, I would know an error is range issue if this logs
        RCLCPP_ERROR(this->get_logger(), "OOB access: box_y=%d box_x=%d", box_y, box_x);
        continue;
      }
      if (this->o_grid[box_y][box_x] < inflated_cost) { // Apply inflated cost if it's larger than current cost
        this->o_grid[box_y][box_x] = inflated_cost;
      } 
    }
  }
}

// LaserScan:
//  - angle_min and angle_max, with angle_increment
//  - ranges (up to 256), distances ordered by angle (starting from min to max with increment of angle_increment)
//  - See how many coordinates are in a grid, if none = zero chance of danger, if > 0 than increased chance
//  - Discard any range out of max range, simple conditional check
void CostmapNode::processLaserMsg(sensor_msgs::msg::LaserScan::SharedPtr laserScan) {

  this->o_grid = initGrid();

  std::vector<std::vector<int>> obstacle_list;

  int origin_x = this->width_cells / 2;
  int origin_y = this->height_cells / 2;

  // Initial o grid marking logic
  // size_t type instead of integer when iterating over size()
  for (size_t i = 0; i < laserScan->ranges.size(); i++) {

    // Make sure distance is not too small / big
    float range = laserScan->ranges[i];

    if (range < laserScan->range_min) {
      continue;
    }

    // Store range validity as a variable
    bool hit = range <= laserScan->range_max;
    float effective_range = hit ? range : laserScan->range_max;

    float angle = laserScan->angle_min + laserScan->angle_increment * i; // 'ranges' starts from smallest angle all the way to max, increasing by angle_increment each item, so with an index i can find the angle
    float x_dis = effective_range * cos(angle); // X and Y distance in meters
    float y_dis = effective_range * sin(angle);
    float grid_res = this->grid_res;
    // static_cast to ensure what I get is an int
    // displacement / grid resolution gives me displacement with respect to the grid's scale, 
    // which i can add to the centre of axis to find the position in grid.
    int cell_x = static_cast<int>((x_dis / grid_res) + this->width_cells / 2); 
    int cell_y = static_cast<int>((y_dis / grid_res) + this->height_cells / 2);

    markFreeLine(origin_y, origin_x, cell_y, cell_x);

    bool in_bounds = cell_x < width_cells && cell_x >= 0 && cell_y < height_cells && cell_y >= 0;

    if (hit && in_bounds) {
      this->o_grid[cell_y][cell_x] = 100;
      obstacle_list.push_back({cell_y, cell_x});
    } else if (!hit && in_bounds && this->o_grid[cell_y][cell_x] != 100) {
      this->o_grid[cell_y][cell_x] = 0;
    }
  }

  // Inflation logic
  // Iterate through marked obstacle, pass their coordinates to inflate cells around them
  for (size_t i = 0; i < obstacle_list.size(); i++) {
    inflateGridCell(obstacle_list[i][0], obstacle_list[i][1], 0, static_cast<int>(this->width_cells - 1));
  }
  
  this->publishCostmapMsg();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>()); // Blocks the thread to keep the node (Costmap) running, starts an event loop for subscription / timer 
  rclcpp::shutdown();
  return 0;
}