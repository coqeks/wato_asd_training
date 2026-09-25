#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

#include <vector>

class CostmapNode : public rclcpp::Node {
public:
    CostmapNode();

    void publishCostmapMsg();
    void inflateGridCell(int cell_y, int cell_x, int min, int max);
    void processLaserMsg(sensor_msgs::msg::LaserScan::SharedPtr);
    void markFreeLine(int y0, int x0, int y1, int x1);

    std::vector<std::vector<int>> initGrid();

    // Grid dimensions
    const double grid_res = 0.1;
    const int grid_width = 20;
    const int grid_height = 20;

    const int width_cells =
        static_cast<int>(grid_width / grid_res);

    const int height_cells =
        static_cast<int>(grid_height / grid_res);

    // Safety inflation radius
    const double inflation_r = 2.0;

    // Occupancy values
    const int max_cost = 100;

    std::vector<std::vector<int>> o_grid;

private:
    robot::CostmapCore costmap_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_;
};

#endif