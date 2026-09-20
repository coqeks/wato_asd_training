#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

 
#include "costmap_core.hpp"

// Declaring node class and its properties
// CostmapNode class is inherited from rclcpp's 'Node' class - https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Node.html
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void publishMessage();
    void publishCostmapMsg();
    void inflateGridCell(int, int, int, int);
    void processLaserMsg(sensor_msgs::msg::LaserScan::SharedPtr); // Declare laser scan subscriber as 
    std::vector<std::vector<int>> initGrid();
    // Shared pointer instead of complete copy, this is the standard apparently

    // in meters
    float grid_res = 0.1; 
    int grid_width = 10;
    int grid_height = 10;
    int width_cells = static_cast<int>(grid_width / grid_res);
    int height_cells = static_cast<int>(grid_height / grid_res);
    float inflation_r = 1.0;
    // Percent
    float max_cost = 100.0;
    std::vector<std::vector<int>> o_grid;
 
  private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_;
};
 
#endif 