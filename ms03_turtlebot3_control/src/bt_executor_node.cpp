#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "ms03_turtlebot3_control/bt_nodes.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <cmath>

// =============================================================
// Sector definitions (ROS2 / REP-103: CCW positive, 0 = forward)
//   Front  : -35° to +35°   (robot nose)
//   Right  : +35° to +145°  (robot's right)
//   Left   : -35° to -145°  (robot's left)
//   Back   : ±145° to ±180°
//
// Obstacle threshold:
//   Front < 0.45m  => obstacle_detected = true => triggers TurnRecovery
// =============================================================

class BTExecutorNode : public rclcpp::Node {
public:
  BTExecutorNode() : Node("bt_executor_node") {
    state_ = std::make_shared<RobotState>();

    // 1. Odometry — Best Effort (ros_gz_bridge in Jazzy uses Best Effort)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        state_->current_pose.header = msg->header;
        state_->current_pose.pose   = msg->pose.pose;
        state_->pose_received       = true;
      });

    // 2. LiDAR — Best Effort (ros_gz_bridge)
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        processScan(msg);
      });

    // 3. Single cmd_vel publisher — SOLE authority, no motion_node conflict
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

    // 4. Live goal update at runtime
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/user_goal", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        tree_.rootBlackboard()->set("target_pose", *msg);
        RCLCPP_INFO(this->get_logger(),
          "[BT] Goal updated via /user_goal: x=%.2f, y=%.2f",
          msg->pose.position.x, msg->pose.position.y);
      });

    // 5. Behavior Tree
    BT::BehaviorTreeFactory factory;

    factory.registerSimpleCondition(
      "GoalCheck",
      [this](BT::TreeNode& self) {
        geometry_msgs::msg::PoseStamped goal;
        if (!self.getInput("goal", goal) || !state_->pose_received) {
          return BT::NodeStatus::FAILURE;
        }
        double dx = goal.pose.position.x - state_->current_pose.pose.position.x;
        double dy = goal.pose.position.y - state_->current_pose.pose.position.y;
        bool reached = (std::hypot(dx, dy) < 0.15);
        if (reached) {
          RCLCPP_INFO(this->get_logger(), "[BT] GOAL REACHED!");
          publishStop();
        }
        return reached ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      },
      {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")});

    // ObstacleCheck returns SUCCESS (=obstacle present) when blocked OR during recovery
    factory.registerSimpleCondition(
      "ObstacleCheck",
      [this](BT::TreeNode&) {
        return (state_->obstacle_detected || state_->recovery_active)
               ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });

    BT::NodeBuilder b_gtp = [this](const std::string& n, const BT::NodeConfig& c) {
      return std::make_unique<GoToPose>(n, c, cmd_pub_, state_);
    };
    factory.registerBuilder<GoToPose>("GoToPose", b_gtp);

    BT::NodeBuilder b_tr = [this](const std::string& n, const BT::NodeConfig& c) {
      return std::make_unique<TurnRecovery>(n, c, cmd_pub_, state_);
    };
    factory.registerBuilder<TurnRecovery>("TurnRecovery", b_tr);

    std::string pkg = ament_index_cpp::get_package_share_directory("ms03_turtlebot3_control");
    tree_ = factory.createTreeFromFile(pkg + "/bt_xml/bt_recovery_tree.xml");

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = 10.0;
    goal.pose.position.y = 7.0;
    tree_.rootBlackboard()->set("target_pose", goal);

    RCLCPP_INFO(this->get_logger(),
      "[BT] Ready. Default goal x=10.0, y=7.0. Publish to /user_goal to change.");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        tree_.tickOnce();
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "[BT] Front: %.2fm | Right(+90°): %.2fm | Left(-90°): %.2fm | Back: %.2fm | Obst: %s | Recov: %s",
          state_->front_dist, state_->right_dist, state_->left_dist, state_->back_dist,
          state_->obstacle_detected ? "YES" : "NO",
          state_->recovery_active   ? "YES" : "NO");
      });
  }

private:
  void processScan(const sensor_msgs::msg::LaserScan::SharedPtr& msg) {
    float min_front = 10.0f, min_right = 10.0f, min_left = 10.0f, min_back = 10.0f;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float r = msg->ranges[i];
      if (std::isnan(r) || std::isinf(r)) continue;
      if (r > 0.0f && r < msg->range_min) r = msg->range_min;   // clamp touching
      if (r <= 0.0f || r > msg->range_max) continue;

      double angle_rad = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      angle_rad = std::atan2(std::sin(angle_rad), std::cos(angle_rad)); // normalize to [-π,π]
      double deg = angle_rad * (180.0 / M_PI);

      // Front: ±45° — robot nose (widened for safer corner clearance)
      if (std::abs(deg) <= 45.0) {
        if (r < min_front) min_front = r;
      }
      // Left: +45° to +125° — narrowed side field of view (left = +90°)
      else if (deg > 45.0 && deg <= 125.0) {
        if (r < min_left) min_left = r;
      }
      // Right: -45° to -125° — narrowed side field of view (right = -90°)
      else if (deg < -45.0 && deg >= -125.0) {
        if (r < min_right) min_right = r;
      }
      // Back: ±145° to ±180°
      else if (std::abs(deg) > 145.0) {
        if (r < min_back) min_back = r;
      }
    }

    state_->front_dist = min_front;
    state_->right_dist = min_right;
    state_->left_dist  = min_left;
    state_->back_dist  = min_back;

    // Only update obstacle_detected when NOT in recovery (recovery manages its own exit)
    // Always update obstacle_detected to ensure it remains reactive
    state_->obstacle_detected = (min_front < 0.30f);
  }

  void publishStop() {
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp    = this->get_clock()->now();
    stop.header.frame_id = "base_link";
    cmd_pub_->publish(stop);
  }

  std::shared_ptr<RobotState> state_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  BT::Tree tree_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BTExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
