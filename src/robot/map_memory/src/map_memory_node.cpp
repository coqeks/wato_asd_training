#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())), global_grid_(this->global_rows_, std::vector<int>(this->global_cols_, -1)) {
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::handleOdomMsg, this, std::placeholders::_1));
  grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::handleGridMsg, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
  publishOccupancyGrid();
}

// Convert orientation data to yaw
static double extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// Calculate position and store
void MapMemoryNode::handleOdomMsg(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
  double x = odomMsg->pose.pose.position.x;
  double y = odomMsg->pose.pose.position.y;

  double x_dis = x - last_x_;
  double y_dis = y - last_y_;

  double distance = std::sqrt(std::pow(x_dis, 2) + std::pow(y_dis, 2));

  // Last x and y are only updated when distance passes distance threshold
  // When displacement is more than threshold, flag to update global map
  if (distance >= distance_threshold) {
    last_x_ = x;
    last_y_ = y;
    costmap_yaw_ = extractYaw(odomMsg->pose.pose.orientation);
    should_update_ = true;
  };
  return;
}

// 
void MapMemoryNode::handleGridMsg(const nav_msgs::msg::OccupancyGrid::SharedPtr gridMsg) {
  latest_costmap_ = *gridMsg;
  costmap_updated_ = true;
  return;
}

void MapMemoryNode::integrateCostMap() {
  // Grid alignment 
  // Costmap's orientation is based on that of the car, while global map's axis is fixed
  // I need to rotate the cells of the costmap to align that of global

  for (int local_row = 0; local_row < costmap_rows; local_row++) {
    for (int local_col = 0; local_col < costmap_cols; local_col++) {

      // Find local x and y from the costmap in metrics
      double local_y = (local_row - (costmap_rows / 2)) * cost_res_;
      double local_x = (local_col - (costmap_cols / 2)) * cost_res_;

      // Rotate the local coordinates to align them with global map, add to the car's coordinates to find global position
      double global_x = last_x_ + (local_x * cos(costmap_yaw_) - local_y * sin(costmap_yaw_));
      double global_y = last_y_ + (local_x * sin(costmap_yaw_) + local_y * cos(costmap_yaw_));

      // Calculate position of global map's origin, then calculate indices relative to origin position
      double origin_x = -(global_cols_ * global_res_) / 2.0;
      double origin_y = -(global_rows_ * global_res_) / 2.0;
      int global_row = std::round((global_y - origin_y) / global_res_);
      int global_col = std::round((global_x - origin_x) / global_res_);

      // Prevent OOB
      if (global_row < 0 || global_row >= global_rows_ || global_col < 0 || global_col >= global_cols_) {
        continue;
      }

      double updated_cost = latest_costmap_.data[local_row * costmap_cols + local_col];
      if (updated_cost == -1) {
        continue;

      // Update if newest cost value is larger, so that map doesn't forget obstacles
      } else if (updated_cost > global_grid_[global_row][global_col]) {
        global_grid_[global_row][global_col] = updated_cost;
      }
    }
  }
  return;
}

void MapMemoryNode::updateMap() {
  if (costmap_updated_ && should_update_) {
    RCLCPP_INFO(this->get_logger(), "Updated Map");
    // Integrate and publish new global map
    integrateCostMap();
    publishOccupancyGrid();
    should_update_ = false;
    costmap_updated_ = false;
  }
  return;
}

void MapMemoryNode::publishOccupancyGrid() {
  global_map_.header.stamp = this->get_clock()->now();
  global_map_.header.frame_id = "map";
  global_map_.info.resolution = global_res_;
  global_map_.info.width = global_cols_;
  global_map_.info.height = global_rows_;
  global_map_.info.origin.position.x = -(global_cols_ * global_res_) / 2.0;
  global_map_.info.origin.position.y = -(global_rows_ * global_res_) / 2.0;
  global_map_.info.origin.orientation.w = 1.0;

  global_map_.data.resize(global_cols_ * global_rows_);
  for (int row = 0; row < global_rows_; row++) {
    for (int col = 0; col < global_cols_; col++) {
      global_map_.data[row * global_cols_ + col] = static_cast<int8_t>(global_grid_[row][col]);
    }
  }

  map_pub_->publish(global_map_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}

