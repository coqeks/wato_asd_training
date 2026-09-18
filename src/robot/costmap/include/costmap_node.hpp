#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
 
#include "costmap_core.hpp"

// Declaring node class and its properties
// CostmapNode class is inherited from rclcpp's 'Node' class - https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Node.html
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void publishMessage();
    void processLaserMsg(sensor_msgs::msg::LaserScan::SharedPtr); // Declare laser scan subscriber as 
    std::vector<std::vector<int>> initGrid();
    // Shared pointer instead of complete copy, this is the standard apparently

    float grid_res = 0.1; // in meters
    int grid_width = 10;
    int grid_height = 10;
 
  private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
#endif 