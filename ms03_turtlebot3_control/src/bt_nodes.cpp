#include "ms03_turtlebot3_control/bt_nodes.hpp"

// ─────────────────────────────────────────────────────────────
// GoToPose
// ─────────────────────────────────────────────────────────────
BT::NodeStatus GoToPose::onStart() {
  return onRunning();
}

BT::NodeStatus GoToPose::onRunning() {
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal) || !state_->pose_received) {
    return BT::NodeStatus::FAILURE;
  }

  double dx = goal.pose.position.x - state_->current_pose.pose.position.x;
  double dy = goal.pose.position.y - state_->current_pose.pose.position.y;
  double dist = std::hypot(dx, dy);

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp    = rclcpp::Clock().now();
  cmd.header.frame_id = "base_link";

  if (dist < 0.15) {
    cmd_pub_->publish(cmd);  // publish zero
    return BT::NodeStatus::SUCCESS;
  }

  tf2::Quaternion q(
    state_->current_pose.pose.orientation.x,
    state_->current_pose.pose.orientation.y,
    state_->current_pose.pose.orientation.z,
    state_->current_pose.pose.orientation.w);
  double r, p, yaw;
  tf2::Matrix3x3(q).getRPY(r, p, yaw);

  double target_yaw = std::atan2(dy, dx);

  // ── Side-proximity heading bias ──────────────────────────
  // If either side is within 0.5m, bias heading 45° (π/4 rad) away
  // from the closer obstacle so the robot steers away gradually.
  //   Left obstacle  → steer right  → negative bias
  //   Right obstacle → steer left   → positive bias
  constexpr float  SIDE_THRESHOLD = 0.20f;
  constexpr double BIAS_RAD       = M_PI / 4.0;  // 45°
  double heading_bias = 0.0;

  bool left_close  = (state_->left_dist  < SIDE_THRESHOLD);
  bool right_close = (state_->right_dist < SIDE_THRESHOLD);

  if (left_close && right_close) {
    // Both sides close — steer away from the tighter side
    if (state_->left_dist <= state_->right_dist) {
      heading_bias = -BIAS_RAD;  // left tighter → steer right
    } else {
      heading_bias = +BIAS_RAD;  // right tighter → steer left
    }
  } else if (left_close) {
    heading_bias = -BIAS_RAD;    // left close → steer right
  } else if (right_close) {
    heading_bias = +BIAS_RAD;    // right close → steer left
  }

  double biased_yaw  = target_yaw + heading_bias;
  double heading_err = std::atan2(std::sin(biased_yaw - yaw), std::cos(biased_yaw - yaw));

  double v = std::clamp(0.4 * dist,        -0.22, 0.22);
  double w = std::clamp(1.5 * heading_err, -1.0,  1.0);

  if (std::abs(heading_err) > 0.4) {
    v *= 0.2;  // slow down while aligning heading
  }

  // Log side bias when active
  if (heading_bias != 0.0) {
    static rclcpp::Clock side_log_clock(RCL_STEADY_TIME);
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("GoToPose"), side_log_clock, 1500,
      "[GoToPose] Side bias %.0f° | L: %.2fm R: %.2fm",
      heading_bias * (180.0 / M_PI),
      state_->left_dist, state_->right_dist);
  }

  cmd.twist.linear.x  = v;
  cmd.twist.angular.z = w;
  cmd_pub_->publish(cmd);

  return BT::NodeStatus::RUNNING;
}

void GoToPose::onHalted() {
  geometry_msgs::msg::TwistStamped stop;
  stop.header.stamp    = rclcpp::Clock().now();
  stop.header.frame_id = "base_link";
  cmd_pub_->publish(stop);
}

// ─────────────────────────────────────────────────────────────
// TurnRecovery
//
// Phase sequence:
//   STOPPING   → full stop  (0.3 s / 3 ticks)
//   REVERSING  → drive back (-0.16 m/s) until front clears or max ticks
//   ROTATING   → rotate to clearer side until front clear
//   BYPASS_STEP→ drive forward briefly to clear corridor
//   → returns SUCCESS, clears recovery_active & obstacle_detected
//
// Clearance thresholds are intentionally lenient so the robot
// doesn't get stuck trying to find perfect clearance:
//   front_clear_threshold  : 0.50 m
//   side_clear_threshold   : 0.28 m  (wide corridors, poles)
// ─────────────────────────────────────────────────────────────
BT::NodeStatus TurnRecovery::onStart() {
  ticks_         = 0;
  rotate_ticks_  = 0;
  bypass_ticks_  = 0;
  phase_         = Phase::STOPPING;
  state_->recovery_active = true;

  // Turn toward the side with MORE clearance
  turn_dir_ = (state_->left_dist >= state_->right_dist) ? 1.0f : -1.0f;

  RCLCPP_INFO(rclcpp::get_logger("TurnRecovery"),
    "[Recovery] START — Front: %.2fm | Left: %.2fm | Right: %.2fm | Turning: %s",
    state_->front_dist, state_->left_dist, state_->right_dist,
    (turn_dir_ > 0) ? "LEFT (CCW)" : "RIGHT (CW)");

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnRecovery::onRunning() {
  state_->recovery_active = true;

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp    = rclcpp::Clock().now();
  cmd.header.frame_id = "base_link";

  // ── PHASE 0: STOPPING ──────────────────────────────────────
  if (phase_ == Phase::STOPPING) {
    ++ticks_;
    // publish zeros
    if (ticks_ >= 3) {
      phase_  = Phase::REVERSING;
      ticks_  = 0;
      RCLCPP_INFO(rclcpp::get_logger("TurnRecovery"), "[Recovery] → REVERSING");
    }
  }

  // ── PHASE 1: REVERSING ─────────────────────────────────────
  else if (phase_ == Phase::REVERSING) {
    ++ticks_;
    cmd.twist.linear.x = -0.16;

    bool behind_blocked  = (state_->back_dist  < 0.22f);
    bool min_done        = (ticks_ >= 6);
    bool max_done        = (ticks_ >= 18);
    // Front cleared enough to start rotating
    bool front_ok = (state_->front_dist >= 0.50f);

    if (behind_blocked || max_done || (min_done && front_ok)) {
      phase_        = Phase::ROTATING;
      rotate_ticks_ = 0;
      // Re-evaluate which side has more room now
      turn_dir_ = (state_->left_dist >= state_->right_dist) ? 1.0f : -1.0f;
      RCLCPP_INFO(rclcpp::get_logger("TurnRecovery"),
        "[Recovery] → ROTATING %s | Front: %.2fm",
        (turn_dir_ > 0) ? "LEFT" : "RIGHT", state_->front_dist);
    }
  }

  // ── PHASE 2: ROTATING ─────────────────────────────────────
  else if (phase_ == Phase::ROTATING) {
    ++rotate_ticks_;
    cmd.twist.angular.z = turn_dir_ * 0.7f;

    // 45° minimum @ 0.7 rad/s, 100ms tick = 0.07 rad/tick → ceil(0.785/0.07) = 12 ticks
    bool min_done  = (rotate_ticks_ >= 12);
    bool max_done  = (rotate_ticks_ >= 45);
    // Front clear threshold set above bypass re-trigger (0.22m)
    bool front_ok  = (state_->front_dist >= 0.30f) &&
                     (state_->left_dist  >= 0.20f || state_->right_dist >= 0.20f);

    if (max_done || (min_done && front_ok)) {
      phase_        = Phase::BYPASS_STEP;
      bypass_ticks_ = 0;
      RCLCPP_INFO(rclcpp::get_logger("TurnRecovery"),
        "[Recovery] → BYPASS_STEP | Front: %.2fm", state_->front_dist);
    }
  }

  // ── PHASE 3: BYPASS_STEP ──────────────────────────────────
  else if (phase_ == Phase::BYPASS_STEP) {
    ++bypass_ticks_;
    cmd.twist.linear.x = 0.18f;

    if (state_->front_dist < 0.22f) {
      // Hit another obstacle during bypass — re-rotate
      phase_        = Phase::ROTATING;
      rotate_ticks_ = 0;
      turn_dir_     = (state_->left_dist >= state_->right_dist) ? 1.0f : -1.0f;
      RCLCPP_WARN(rclcpp::get_logger("TurnRecovery"),
        "[Recovery] Obstacle during bypass (%.2fm), re-rotating", state_->front_dist);
    } else if (bypass_ticks_ >= 10) {
      // Recovery complete — let BT return to GoToPose
      state_->recovery_active  = false;
      state_->obstacle_detected = false;
      cmd_pub_->publish(cmd);   // publish final step
      RCLCPP_INFO(rclcpp::get_logger("TurnRecovery"),
        "[Recovery] COMPLETE — resuming navigation");
      return BT::NodeStatus::SUCCESS;
    }
  }

  cmd_pub_->publish(cmd);
  return BT::NodeStatus::RUNNING;
}

void TurnRecovery::onHalted() {
  state_->recovery_active = false;
  geometry_msgs::msg::TwistStamped stop;
  stop.header.stamp    = rclcpp::Clock().now();
  stop.header.frame_id = "base_link";
  cmd_pub_->publish(stop);
}
