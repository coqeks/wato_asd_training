#include <chrono>
#include <cmath>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::handleMapMsg, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::handleGoalMsg, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::handleOdomMsg, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::checkGoalStatus, this));

  last_plan_time_ = this->get_clock()->now();
}

// New map arrived, only worth replanning if we're actually mid-navigation
void PlannerNode::handleMapMsg(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg) {
  this->current_map_ = *mapMsg;
  this->map_received_ = true;

  if (this->state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

// Switch state and get to planning path
void PlannerNode::handleGoalMsg(const geometry_msgs::msg::PointStamped::SharedPtr goalMsg) {
  this->goal_ = *goalMsg;
  this->goal_received_ = true;
  this->state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

// Just track position, used by goalReached() and worldToGrid() when planning
void PlannerNode::handleOdomMsg(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
  this->robot_x_ = odomMsg->pose.pose.position.x;
  this->robot_y_ = odomMsg->pose.pose.position.y;
}

// Timer callback, checks if we've reached the goal or need to replan due to timeout
void PlannerNode::checkGoalStatus() {
  if (this->state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    this->state_ = State::WAITING_FOR_GOAL;
    return;
  }

  double seconds_since_plan = (this->get_clock()->now() - this->last_plan_time_).seconds();
  if (seconds_since_plan >= this->replan_timeout) {
    RCLCPP_WARN(this->get_logger(), "Replan timeout reached, replanning...");
    planPath();
  }
}

bool PlannerNode::goalReached() {
  double dx = this->goal_.point.x - this->robot_x_;
  double dy = this->goal_.point.y - this->robot_y_;
  return std::sqrt(dx * dx + dy * dy) < this->goal_tolerance;
}

// Convert a world coordinate (meters) into a grid cell, using the map's own resolution and origin
// Returns false if the resulting cell falls outside the map bounds
bool PlannerNode::worldToGrid(double wx, double wy, CellIndex &out) {
  double res = this->current_map_.info.resolution;
  double origin_x = this->current_map_.info.origin.position.x;
  double origin_y = this->current_map_.info.origin.position.y;

  out.x = static_cast<int>((wx - origin_x) / res);
  out.y = static_cast<int>((wy - origin_y) / res);

  bool in_bounds = out.x >= 0 && out.x < static_cast<int>(this->current_map_.info.width) &&
                    out.y >= 0 && out.y < static_cast<int>(this->current_map_.info.height);
  return in_bounds;
}

// Reverse of worldToGrid, used when building the path message to publish
void PlannerNode::gridToWorld(const CellIndex &cell, double &wx, double &wy) {
  double res = this->current_map_.info.resolution;
  double origin_x = this->current_map_.info.origin.position.x;
  double origin_y = this->current_map_.info.origin.position.y;

  // + 0.5 to land in the middle of the cell instead of the corner
  wx = origin_x + (cell.x + 0.5) * res;
  wy = origin_y + (cell.y + 0.5) * res;
}

// -1 (unknown) is treated as safe to plan through, since the robot hasn't seen it yet
// Anything at or above occupancy_threshold is treated as blocked
bool PlannerNode::isTraversable(const CellIndex &cell) {
  if (cell.x < 0 || cell.x >= static_cast<int>(this->current_map_.info.width) ||
      cell.y < 0 || cell.y >= static_cast<int>(this->current_map_.info.height)) {
    return false;
  }

  int index = cell.y * this->current_map_.info.width + cell.x;
  int8_t value = this->current_map_.data[index];

  if (value == -1) {
    return true;
  }

  return value < this->occupancy_threshold;
}

// Euclidean distance in cell units, used as the A* heuristic
double PlannerNode::heuristic(const CellIndex &a, const CellIndex &b) {
  double dx = static_cast<double>(a.x - b.x);
  double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

// Walk backwards through came_from starting at goal_cell, then flip it around
std::vector<CellIndex> PlannerNode::reconstructPath(
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> &came_from,
  CellIndex &start, CellIndex &goal_cell) {

  std::vector<CellIndex> path;
  CellIndex current = goal_cell;

  while (current != start) {
    path.push_back(current);
    auto it = came_from.find(current);
    if (it == came_from.end()) {
      // Shouldn't happen if found == true, but guard anyway
      return std::vector<CellIndex>();
    }
    current = it->second;
  }
  path.push_back(start);

  // Reverse manually rather than pulling in <algorithm> for one call
  std::vector<CellIndex> reversed_path;
  for (int i = static_cast<int>(path.size()) - 1; i >= 0; i--) {
    reversed_path.push_back(path[i]);
  }

  return reversed_path;
}

// OccupancyGrid:
//  - info.width, info.height, info.resolution, info.origin: describe how to convert cell <-> world coords
//  - data: flattened row-major array, -1 unknown, 0 free, up to 100 occupied
void PlannerNode::planPath() {
  if (!this->goal_received_ || !this->map_received_ || this->current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: missing map or goal!");
    return;
  }

  CellIndex start, goal_cell;
  bool start_in_bounds = worldToGrid(this->robot_x_, this->robot_y_, start);
  bool goal_in_bounds = worldToGrid(this->goal_.point.x, this->goal_.point.y, goal_cell);

  if (!start_in_bounds || !goal_in_bounds) {
    RCLCPP_WARN(this->get_logger(), "Start or goal falls outside the current map");
    return;
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  g_score[start] = 0.0;
  open_set.push(AStarNode(start, heuristic(start, goal_cell)));

  // 8 directions, diagonals included, cardinal cost 1, diagonal cost sqrt(2)
  std::vector<CellIndex> directions;
  directions.push_back(CellIndex(1, 0));
  directions.push_back(CellIndex(-1, 0));
  directions.push_back(CellIndex(0, 1));
  directions.push_back(CellIndex(0, -1));
  directions.push_back(CellIndex(1, 1));
  directions.push_back(CellIndex(1, -1));
  directions.push_back(CellIndex(-1, 1));
  directions.push_back(CellIndex(-1, -1));

  bool found = false;

  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();

    if (current == goal_cell) {
      found = true;
      break;
    }

    for (size_t i = 0; i < directions.size(); i++) {
      CellIndex neighbor(current.x + directions[i].x, current.y + directions[i].y);

      if (!isTraversable(neighbor)) {
        continue;
      }

      bool is_diagonal = directions[i].x != 0 && directions[i].y != 0;
      double step_cost = is_diagonal ? std::sqrt(2.0) : 1.0;
      double tentative_g = g_score[current] + step_cost;

      auto it = g_score.find(neighbor);
      if (it == g_score.end() || tentative_g < it->second) {
        g_score[neighbor] = tentative_g;
        came_from[neighbor] = current;
        double f = tentative_g + heuristic(neighbor, goal_cell);
        open_set.push(AStarNode(neighbor, f));
      }
    }
  }

  this->last_plan_time_ = this->get_clock()->now();

  if (!found) {
    RCLCPP_WARN(this->get_logger(), "No valid path found to goal");
    return;
  }

  std::vector<CellIndex> cell_path = reconstructPath(came_from, start, goal_cell);

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->get_clock()->now();
  path_msg.header.frame_id = "map";

  for (size_t i = 0; i < cell_path.size(); i++) {
    geometry_msgs::msg::PoseStamped pose;
    double wx, wy;
    gridToWorld(cell_path[i], wx, wy);
    pose.header = path_msg.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  path_pub_->publish(path_msg);
  RCLCPP_INFO(this->get_logger(), "Path published with %zu waypoints", cell_path.size());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}