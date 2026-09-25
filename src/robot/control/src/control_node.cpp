#include <cmath>

#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&ControlNode::handlePathMsg, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&ControlNode::handleOdomMsg, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::handlePathMsg(const nav_msgs::msg::Path::SharedPtr pathMsg) {
  this->current_path_ = *pathMsg;
  this->path_received_ = true;
}

static double extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void ControlNode::handleOdomMsg(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
  this->robot_x_ = odomMsg->pose.pose.position.x;
  this->robot_y_ = odomMsg->pose.pose.position.y;
  this->robot_yaw_ = extractYaw(odomMsg->pose.pose.orientation);
}

double ControlNode::computeDistance(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

bool ControlNode::closeToGoal() {
  if (this->current_path_.poses.empty()) {
    return false;
  }
  const auto &last_pose = this->current_path_.poses.back();
  double dist = computeDistance(this->robot_x_, this->robot_y_, last_pose.pose.position.x, last_pose.pose.position.y);
  return dist < this->goal_tolerance;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  if (this->current_path_.poses.empty()) {
    return std::nullopt;
  }

  for (size_t i = 0; i < this->current_path_.poses.size(); i++) {
    double px = this->current_path_.poses[i].pose.position.x;
    double py = this->current_path_.poses[i].pose.position.y;
    double dist = computeDistance(this->robot_x_, this->robot_y_, px, py);

    if (dist >= this->lookahead_distance) {
      return this->current_path_.poses[i];
    }
  }

  return this->current_path_.poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  geometry_msgs::msg::Twist cmd_vel;

  double dx = target.pose.position.x - this->robot_x_;
  double dy = target.pose.position.y - this->robot_y_;

  double local_x = dx * cos(this->robot_yaw_) + dy * sin(this->robot_yaw_);
  double local_y = -dx * sin(this->robot_yaw_) + dy * cos(this->robot_yaw_);

  double lookahead_sq = local_x * local_x + local_y * local_y;

  if (lookahead_sq < 1e-6) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
  }

  double curvature = (2.0 * local_y) / lookahead_sq;

  cmd_vel.linear.x = this->linear_speed;
  cmd_vel.angular.z = this->linear_speed * curvature;

  return cmd_vel;
}

void ControlNode::controlLoop() {
  if (!this->path_received_ || this->current_path_.poses.empty()) {
    return;
  }

  if (closeToGoal()) {
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    return;
  }

  geometry_msgs::msg::Twist cmd_vel = computeVelocity(*lookahead_point);
  cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}