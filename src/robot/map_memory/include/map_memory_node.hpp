#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

    // Callbacks
    void publishOccupancyGrid();
    void handleOdomMsg(nav_msgs::msg::Odometry::SharedPtr);
    void handleGridMsg(nav_msgs::msg::OccupancyGrid::SharedPtr);
    void integrateCostMap();
    void updateMap();

  private:
    robot::MapMemoryCore map_memory_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid latest_costmap_;
    nav_msgs::msg::OccupancyGrid global_map_;

    // In meters
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double costmap_yaw_; // Used to align costmap axis to global
    const double distance_threshold = 0.5;
    const int costmap_cols = 200;
    const int costmap_rows = 200;
    bool costmap_updated_ = false; // In pseudo code, this is always true after the very first costmap msg, probably should flag it false after each update
    bool should_update_ = false;
    const double global_res_ = 0.15; // Global resolution is lower than costmap (0.1 m) so that global map surely has every cells marked by costmap
    const int global_cols_ = 200;
    const int global_rows_ = 200;

    const double cost_res_ = 0.1;

    std::vector<std::vector<int>> global_grid_;

};

#endif 
