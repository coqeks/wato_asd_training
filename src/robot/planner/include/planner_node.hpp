#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <vector>
#include <queue>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "planner_core.hpp"

struct CellIndex
{
  int x;
  int y;

  CellIndex(int x_val, int y_val) : x(x_val), y(y_val) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode
{
  CellIndex index;
  double f_score;

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // Smallest f_score should be on top of the priority queue
    return a.f_score > b.f_score;
  }
};

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

    // Callbacks
    void handleMapMsg(nav_msgs::msg::OccupancyGrid::SharedPtr);
    void handleGoalMsg(geometry_msgs::msg::PointStamped::SharedPtr);
    void handleOdomMsg(nav_msgs::msg::Odometry::SharedPtr);
    void checkGoalStatus();
    void planPath();

  private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

    // Helpers, used inside planPath
    bool worldToGrid(double wx, double wy, CellIndex &out);
    void gridToWorld(const CellIndex &cell, double &wx, double &wy);
    bool isTraversable(const CellIndex &cell);
    double heuristic(const CellIndex &a, const CellIndex &b);
    bool goalReached();
    std::vector<CellIndex> reconstructPath(
      std::unordered_map<CellIndex, CellIndex, CellIndexHash> &came_from,
      CellIndex &start, CellIndex &goal_cell);

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::WAITING_FOR_GOAL;

    nav_msgs::msg::OccupancyGrid current_map_;
    bool map_received_ = false;

    geometry_msgs::msg::PointStamped goal_;
    bool goal_received_ = false;

    // in meters, updated every time odom comes in
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;

    rclcpp::Time last_plan_time_;

    // below within this = goal reached, past below = time to replan
    const double goal_tolerance = 0.5;
    // anything above this cost is treated as blocked, -1 (unknown) is handled separately
    const int occupancy_threshold = 12.5;
    const double replan_timeout = 5.0;
};

#endif