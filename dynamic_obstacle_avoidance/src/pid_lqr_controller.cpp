#include <cmath>
#include <limits>
#include <memory>

#include "dynamic_obstacle_avoidance/pid_lqr_controller.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/time.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

PLUGINLIB_EXPORT_CLASS(
  dynamic_obstacle_avoidance::PidLqrController,
  nav2_core::Controller)

namespace dynamic_obstacle_avoidance
{

void PidLqrController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  auto node = parent.lock();
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kp_angular", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".ki_angular", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kd_angular", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lqr_k1", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lqr_k2", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".target_linear_velocity", rclcpp::ParameterValue(0.22));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".final_approach_tolerance", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".yaw_align_tolerance", rclcpp::ParameterValue(0.08));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kp_final_yaw", rclcpp::ParameterValue(2.5));

  node->get_parameter(plugin_name_ + ".kp_angular", kp_angular_);
  node->get_parameter(plugin_name_ + ".ki_angular", ki_angular_);
  node->get_parameter(plugin_name_ + ".kd_angular", kd_angular_);
  node->get_parameter(plugin_name_ + ".lqr_k1", lqr_k1_);
  node->get_parameter(plugin_name_ + ".lqr_k2", lqr_k2_);
  node->get_parameter(plugin_name_ + ".target_linear_velocity", target_linear_velocity_);
  node->get_parameter(plugin_name_ + ".final_approach_tolerance", final_approach_tolerance_);
  node->get_parameter(plugin_name_ + ".yaw_align_tolerance", yaw_align_tolerance_);
  node->get_parameter(plugin_name_ + ".kp_final_yaw", kp_final_yaw_);

  clock_ = node->get_clock();
  RCLCPP_INFO(
    node->get_logger(),
    "PID+LQR Controller [%s] initialized (kp=%.2f ki=%.2f kd=%.2f k1=%.2f k2=%.2f v=%.2f)",
    plugin_name_.c_str(), kp_angular_, ki_angular_, kd_angular_, lqr_k1_, lqr_k2_,
    target_linear_velocity_);
}

void PidLqrController::cleanup()
{
  global_plan_.poses.clear();
  plan_initialized_ = false;
  current_waypoint_idx_ = 0;
  angular_integral_ = 0.0;
  last_angle_error_ = 0.0;
}

void PidLqrController::activate() {}

void PidLqrController::deactivate() {}

void PidLqrController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_.poses.clear();
  current_waypoint_idx_ = 0;
  angular_integral_ = 0.0;
  last_angle_error_ = 0.0;
  plan_initialized_ = false;

  if (path.poses.empty() || !tf_ || !costmap_ros_) {
    if (auto node = node_.lock()) {
      RCLCPP_WARN(node->get_logger(), "[%s] setPlan: empty path or missing tf/costmap", plugin_name_.c_str());
    }
    return;
  }

  // Robot pose in computeVelocityCommands() is in the LOCAL costmap frame
  // (e.g. "odom"), while the plan arrives in the GLOBAL frame (e.g. "map").
  // Transform every waypoint once so both are in the same frame.
  const std::string target_frame = costmap_ros_->getGlobalFrameID();
  auto node = node_.lock();
  rclcpp::Time now = node->now();

  global_plan_.header = path.header;
  global_plan_.header.frame_id = target_frame;

  geometry_msgs::msg::PoseStamped wp;
  for (const auto & p : path.poses) {
    wp.pose = p.pose;
    wp.header = p.header;
    if (wp.header.frame_id.empty()) {
      wp.header.frame_id = path.header.frame_id;
    }
    wp.header.stamp = now;
    try {
      geometry_msgs::msg::PoseStamped transformed = tf_->transform(
        wp, target_frame, tf2::durationFromSec(0.2));
      transformed.header.frame_id = target_frame;
      global_plan_.poses.push_back(transformed);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "[%s] setPlan: cannot transform waypoint into %s: %s",
        plugin_name_.c_str(), target_frame.c_str(), ex.what());
    }
  }

  if (!global_plan_.poses.empty()) {
    plan_initialized_ = true;
    global_frame_ = target_frame;
  }
}

double PidLqrController::normalize_angle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

geometry_msgs::msg::TwistStamped PidLqrController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker *)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;
  cmd.twist.linear.x = 0.0;
  cmd.twist.angular.z = 0.0;

  if (!plan_initialized_ || global_plan_.poses.empty()) {
    return cmd;
  }

  double rx = pose.pose.position.x;
  double ry = pose.pose.position.y;

  tf2::Quaternion q(
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z,
    pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, r_theta;
  m.getRPY(roll, pitch, r_theta);

  auto & target_pose = global_plan_.poses[current_waypoint_idx_].pose;
  double tx = target_pose.position.x;
  double ty = target_pose.position.y;

  double dx = tx - rx;
  double dy = ty - ry;
  double distance_to_target = std::sqrt(dx * dx + dy * dy);

  if (distance_to_target < waypoint_tolerance_) {
    if (current_waypoint_idx_ < global_plan_.poses.size() - 1) {
      current_waypoint_idx_++;
      if (auto node = node_.lock()) {
        RCLCPP_DEBUG(
          node->get_logger(), "[%s] switched to waypoint %zu",
          plugin_name_.c_str(), current_waypoint_idx_);
      }
    } else if (distance_to_target <= final_approach_tolerance_) {
      // Final waypoint within approach radius: rotate in place to the goal
      // orientation so the nav2 goal checker (xy + yaw) can declare success.
      tf2::Quaternion qg(
        target_pose.orientation.x, target_pose.orientation.y,
        target_pose.orientation.z, target_pose.orientation.w);
      tf2::Matrix3x3 mg(qg);
      double groll, gpitch, goal_yaw;
      mg.getRPY(groll, gpitch, goal_yaw);
      double yaw_err = normalize_angle(goal_yaw - r_theta);
      if (std::fabs(yaw_err) > yaw_align_tolerance_) {
        double av = std::max(
          -max_angular_vel_, std::min(max_angular_vel_, kp_final_yaw_ * yaw_err));
        cmd.twist.angular.z = av;
        return cmd;
      }
      return cmd;  // aligned with goal: stop
    }
  }

  target_pose = global_plan_.poses[current_waypoint_idx_].pose;
  tx = target_pose.position.x;
  ty = target_pose.position.y;
  dx = tx - rx;
  dy = ty - ry;
  distance_to_target = std::sqrt(dx * dx + dy * dy);

  double target_theta = std::atan2(dy, dx);
  double angle_error = normalize_angle(target_theta - r_theta);

  double angular_vel =
    (kp_angular_ * angle_error) +
    (ki_angular_ * angular_integral_) +
    (kd_angular_ * (angle_error - last_angle_error_) / control_period_);
  angular_vel = std::max(-max_angular_vel_, std::min(max_angular_vel_, angular_vel));

  angular_integral_ += angle_error * control_period_;
  last_angle_error_ = angle_error;

  double target_v = target_linear_velocity_;
  if (use_speed_limit_) {
    target_v = std::min(target_v, speed_limit_);
  }
  // Slow down when approaching the final waypoint so the robot settles
  // (and can rotate to the goal heading) without overshooting.
  if (current_waypoint_idx_ == global_plan_.poses.size() - 1) {
    target_v *= std::min(1.0, distance_to_target / waypoint_tolerance_);
  }
  double current_v = velocity.linear.x;
  double pos_error = distance_to_target;
  double vel_error = target_v - current_v;

  double linear_vel = target_v + (lqr_k1_ * pos_error + lqr_k2_ * vel_error);
  linear_vel = std::max(0.0, std::min(linear_vel, target_v));

  if (std::abs(angle_error) > 0.5) {
    linear_vel *= 0.1;
  }

  cmd.twist.linear.x = linear_vel;
  cmd.twist.angular.z = angular_vel;
  return cmd;
}

void PidLqrController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit == 0.0) {
    return;
  }
  if (percentage) {
    speed_limit_ = speed_limit * target_linear_velocity_;
  } else {
    speed_limit_ = speed_limit;
  }
  use_speed_limit_ = true;
}

}  // namespace dynamic_obstacle_avoidance
