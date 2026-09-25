#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <vector>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

    // Callbacks
    void handlePathMsg(nav_msgs::msg::Path::SharedPtr);
    void handleOdomMsg(nav_msgs::msg::Odometry::SharedPtr);
    void controlLoop();

  private:
    // Helpers, used inside controlLoop
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target);
    double computeDistance(double x1, double y1, double x2, double y2);
    bool closeToGoal();

    robot::ControlCore control_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path current_path_;
    bool path_received_ = false;

    // In meters, updated every time odom comes in
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;

    // Tuning parameters
    const double lookahead_distance = 1.0;
    const double goal_tolerance = 0.2;
    const double linear_speed = 0.5;
};

#endif