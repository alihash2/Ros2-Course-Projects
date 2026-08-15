#ifndef MS03_TURTLEBOT3_CONTROL__BT_NODES_HPP_
#define MS03_TURTLEBOT3_CONTROL__BT_NODES_HPP_

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <memory>

struct RobotState {
  geometry_msgs::msg::PoseStamped current_pose;
  bool obstacle_detected{false};
  bool recovery_active{false};
  bool pose_received{false};
  float front_dist{10.0f};
  float left_dist{10.0f};
  float right_dist{10.0f};
  float back_dist{10.0f};
};

class GoToPose : public BT::StatefulActionNode {
public:
  GoToPose(const std::string& name, const BT::NodeConfig& config,
           rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub,
           std::shared_ptr<RobotState> state)
    : BT::StatefulActionNode(name, config), cmd_pub_(cmd_pub), state_(state) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("goal") };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  std::shared_ptr<RobotState> state_;
};

class TurnRecovery : public BT::StatefulActionNode {
public:
  enum class Phase { STOPPING, REVERSING, ROTATING, BYPASS_STEP };

  TurnRecovery(const std::string& name, const BT::NodeConfig& config,
               rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub,
               std::shared_ptr<RobotState> state)
    : BT::StatefulActionNode(name, config), cmd_pub_(pub), state_(state) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  std::shared_ptr<RobotState> state_;
  Phase phase_{Phase::REVERSING};
  int ticks_{0};
  int rotate_ticks_{0};
  int bypass_ticks_{0};
  float turn_dir_{1.0f};
};

#endif  // MS03_TURTLEBOT3_CONTROL__BT_NODES_HPP_