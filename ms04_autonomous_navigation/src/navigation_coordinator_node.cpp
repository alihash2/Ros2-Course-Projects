#include "ms04_autonomous_navigation/navigation_coordinator_node.hpp"

#include <cmath>
#include <functional>

#include "builtin_interfaces/msg/duration.hpp"

namespace ms04_autonomous_navigation {

NavigationCoordinatorNode::NavigationCoordinatorNode(const rclcpp::NodeOptions & options)
    : Node("navigation_coordinator_node", options),
      state_(MissionState::IDLE),
      cancel_reason_(CancelReason::NONE),
      mission_current_recoveries_(0),
      have_pose_(false),
      average_speed_(0.3) {
    // ---- Parameters ----
    this->declare_parameter<std::string>("navigate_to_pose_action", "/navigate_to_pose");
    this->declare_parameter<std::string>("follow_waypoints_action", "/follow_waypoints");
    this->declare_parameter<std::string>("execute_mission_action", "/execute_mission");
    this->declare_parameter<double>("average_speed", 0.3);

    navigate_to_pose_action_ = this->get_parameter("navigate_to_pose_action").as_string();
    follow_waypoints_action_ = this->get_parameter("follow_waypoints_action").as_string();
    execute_mission_action_ = this->get_parameter("execute_mission_action").as_string();
    average_speed_ = this->get_parameter("average_speed").as_double();

    // ---- Action clients to Nav2 ----
    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, navigate_to_pose_action_);
    follow_waypoints_client_ = rclcpp_action::create_client<FollowWaypoints>(
        this, follow_waypoints_action_);

    // ---- Publishers ----
    event_pub_ = this->create_publisher<NavigationEvent>("/navigation/events", 10);
    status_pub_ = this->create_publisher<NavigationStatus>("/navigation/status", 10);
    waypoint_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/navigation/waypoints_marker", 10);

    // ---- Mission command dispatcher ----
    mission_sub_ = this->create_subscription<NavigationMission>(
        "/navigation/mission", 10,
        std::bind(&NavigationCoordinatorNode::mission_command_callback, this,
                  std::placeholders::_1));

    // ---- Odom for distance estimates during waypoint missions ----
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&NavigationCoordinatorNode::odom_callback, this, std::placeholders::_1));

    // ---- ExecuteMission action server ----
    execute_mission_server_ = rclcpp_action::create_server<ExecuteMission>(
        this, execute_mission_action_,
        std::bind(&NavigationCoordinatorNode::mission_goal_callback, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&NavigationCoordinatorNode::mission_cancel_callback, this,
                  std::placeholders::_1),
        std::bind(&NavigationCoordinatorNode::mission_accepted_callback, this,
                  std::placeholders::_1));

    publish_status(MissionState::IDLE, "coordinator ready");
    RCLCPP_INFO(this->get_logger(), "NavigationCoordinatorNode ready (action: %s / %s)",
                navigate_to_pose_action_.c_str(), follow_waypoints_action_.c_str());
}

// ---------------------------------------------------------------------------
// Command dispatcher
// ---------------------------------------------------------------------------

void NavigationCoordinatorNode::mission_command_callback(
    const NavigationMission::SharedPtr msg) {
    process_command(msg->command, msg->mode, msg);
}

void NavigationCoordinatorNode::process_command(
    uint8_t command, uint8_t mode, const NavigationMission::SharedPtr msg) {
    (void)mode;
    switch (command) {
        case NavigationMission::COMMAND_START:
            start_mission(msg);
            break;
        case NavigationMission::COMMAND_CANCEL:
            handle_cancel();
            break;
        case NavigationMission::COMMAND_PAUSE:
            handle_pause();
            break;
        case NavigationMission::COMMAND_RESUME:
            handle_resume();
            break;
        case NavigationMission::COMMAND_REPLACE:
            handle_replace(msg);
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Unknown command %u", command);
            break;
    }
}

void NavigationCoordinatorNode::start_mission(const NavigationMission::SharedPtr msg) {
    if (state_ == MissionState::NAVIGATING || state_ == MissionState::PAUSED) {
        std::string reason =
            "start rejected: mission already active (" + mission_.mission_id + ")";
        publish_status(MissionState::ABORTED, reason);
        publish_navigation_event(
            NavigationEvent::EVENT_GOAL_REJECTED, reason, current_pose_, 0.0f,
            builtin_interfaces::msg::Duration(), 0, 0, mission_.total_waypoints);
        RCLCPP_WARN(this->get_logger(), "%s", reason.c_str());
        return;
    }

    MissionContext ctx;
    ctx.mission_id = msg->mission_id;
    ctx.mode = msg->mode;
    ctx.target_pose = msg->target_pose;
    ctx.waypoints = msg->waypoints;
    ctx.remaining_waypoints = msg->waypoints;
    ctx.current_waypoint = 0;
    ctx.total_waypoints = ctx.waypoints.size();
    ctx.distance_remaining = 0.0f;

    if (ctx.mode == NavigationMission::MODE_GO_TO_POSE) {
        ctx.total_waypoints = 1;
    }

    dispatch_mission(ctx);
}

void NavigationCoordinatorNode::replace_mission(const NavigationMission::SharedPtr msg) {
    if (state_ == MissionState::NAVIGATING) {
        cancel_reason_ = CancelReason::REPLACE;
        pending_mission_ = msg;
        publish_status(MissionState::ABORTED, "replacing active goal");
        publish_navigation_event(
            NavigationEvent::EVENT_GOAL_REPLACED,
            "replacing active mission with " + msg->mission_id, current_pose_,
            0.0f, builtin_interfaces::msg::Duration(), 0, 0,
            mission_.total_waypoints);
        RCLCPP_INFO(this->get_logger(), "replace requested, cancelling current goal");
        if (active_nav_handle_) {
            navigate_to_pose_client_->async_cancel_goal(active_nav_handle_);
        }
        if (active_follow_handle_) {
            follow_waypoints_client_->async_cancel_goal(active_follow_handle_);
        }
    } else {
        // idle / paused -> dispatch immediately
        RCLCPP_INFO(this->get_logger(), "replace requested (no active goal)");
        cancel_reason_ = CancelReason::NONE;
        pending_mission_ = nullptr;
        start_mission(msg);
    }
}

void NavigationCoordinatorNode::dispatch_mission(const MissionContext & ctx) {
    mission_ = ctx;
    state_ = MissionState::NAVIGATING;
    publish_waypoint_markers();

    publish_navigation_event(
        NavigationEvent::EVENT_GOAL_SUBMITTED,
        "mission submitted: " + mission_.mission_id, current_pose_, 0.0f,
        builtin_interfaces::msg::Duration(), 0, 0, mission_.total_waypoints);

    if (mission_.mode == NavigationMission::MODE_GO_TO_POSE) {
        publish_status(MissionState::NAVIGATING,
                       "executing go-to-pose mission " + mission_.mission_id);
        dispatch_navigate_to_pose(mission_.target_pose);
    } else {
        publish_status(MissionState::NAVIGATING,
                       "executing waypoint mission " + mission_.mission_id);
        dispatch_follow_waypoints(mission_.remaining_waypoints);
    }
}

void NavigationCoordinatorNode::dispatch_navigate_to_pose(
    const geometry_msgs::msg::PoseStamped & pose) {
    if (!navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(2))) {
        publish_status(MissionState::ABORTED, "navigate_to_pose action server unavailable");
        RCLCPP_ERROR(this->get_logger(), "navigate_to_pose action server unavailable");
        finish_mission(NavigationEvent::EVENT_GOAL_ABORTED, false, "navigate_to_pose action server unavailable");
        return;
    }

    auto goal = std::make_shared<NavigateToPose::Goal>();
    goal->pose = pose;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NavigationCoordinatorNode::nav_goal_response, this,
                  std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NavigationCoordinatorNode::nav_feedback, this,
                  std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&NavigationCoordinatorNode::nav_result, this, std::placeholders::_1);

    navigate_to_pose_client_->async_send_goal(*goal, send_goal_options);
}

void NavigationCoordinatorNode::dispatch_follow_waypoints(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses) {
    if (poses.empty()) {
        publish_status(MissionState::ABORTED, "waypoint list is empty");
        RCLCPP_ERROR(this->get_logger(), "waypoint mission with empty list");
        finish_mission(NavigationEvent::EVENT_GOAL_ABORTED, false, "waypoint list is empty");
        return;
    }

    if (!follow_waypoints_client_->wait_for_action_server(std::chrono::seconds(2))) {
        publish_status(MissionState::ABORTED, "follow_waypoints action server unavailable");
        RCLCPP_ERROR(this->get_logger(), "follow_waypoints action server unavailable");
        finish_mission(NavigationEvent::EVENT_GOAL_ABORTED, false, "follow_waypoints action server unavailable");
        return;
    }

    auto goal = std::make_shared<FollowWaypoints::Goal>();
    goal->poses = poses;

    auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NavigationCoordinatorNode::follow_goal_response, this,
                  std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NavigationCoordinatorNode::follow_feedback, this,
                  std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&NavigationCoordinatorNode::follow_result, this, std::placeholders::_1);

    follow_waypoints_client_->async_send_goal(*goal, send_goal_options);
}

// ---------------------------------------------------------------------------
// Control handlers
// ---------------------------------------------------------------------------

void NavigationCoordinatorNode::handle_cancel() {
    if (state_ == MissionState::IDLE) {
        publish_status(MissionState::IDLE, "cancel ignored: no active mission");
        RCLCPP_WARN(this->get_logger(), "cancel ignored: no active mission");
        return;
    }
    if (state_ == MissionState::PAUSED) {
        publish_status(MissionState::CANCELED, "mission canceled while paused");
        finish_mission(
            NavigationEvent::EVENT_GOAL_CANCELED, false, "mission canceled while paused");
        return;
    }
    cancel_reason_ = CancelReason::USER_CANCEL;
    RCLCPP_INFO(this->get_logger(), "cancel requested");
    if (active_nav_handle_) {
        navigate_to_pose_client_->async_cancel_goal(active_nav_handle_);
    }
    if (active_follow_handle_) {
        follow_waypoints_client_->async_cancel_goal(active_follow_handle_);
    }
}

void NavigationCoordinatorNode::handle_pause() {
    if (state_ != MissionState::NAVIGATING) {
        publish_status(state_, "pause ignored: no active goal");
        RCLCPP_WARN(this->get_logger(), "pause ignored: no active goal");
        return;
    }
    cancel_reason_ = CancelReason::PAUSE;
    publish_status(MissionState::PAUSED, "pausing mission " + mission_.mission_id);
    publish_navigation_event(
        NavigationEvent::EVENT_GOAL_PAUSED, "pausing mission " + mission_.mission_id,
        current_pose_, 0.0f, builtin_interfaces::msg::Duration(), 0,
        mission_.current_waypoint, mission_.total_waypoints);
    RCLCPP_INFO(this->get_logger(), "pause requested");
    if (active_nav_handle_) {
        navigate_to_pose_client_->async_cancel_goal(active_nav_handle_);
    }
    if (active_follow_handle_) {
        follow_waypoints_client_->async_cancel_goal(active_follow_handle_);
    }
}

void NavigationCoordinatorNode::handle_resume() {
    if (state_ != MissionState::PAUSED) {
        publish_status(state_, "resume ignored: not paused");
        RCLCPP_WARN(this->get_logger(), "resume ignored: not paused");
        return;
    }
    publish_status(MissionState::NAVIGATING, "resuming mission " + mission_.mission_id);
    publish_navigation_event(
        NavigationEvent::EVENT_GOAL_RESUMED, "resuming mission " + mission_.mission_id,
        current_pose_, 0.0f, builtin_interfaces::msg::Duration(), 0,
        mission_.current_waypoint, mission_.total_waypoints);
    RCLCPP_INFO(this->get_logger(), "resume requested");
    if (mission_.mode == NavigationMission::MODE_GO_TO_POSE) {
        dispatch_navigate_to_pose(mission_.target_pose);
    } else {
        dispatch_follow_waypoints(mission_.remaining_waypoints);
    }
}

void NavigationCoordinatorNode::handle_replace(const NavigationMission::SharedPtr msg) {
    replace_mission(msg);
}

// ---------------------------------------------------------------------------
// NavigateToPose callbacks
// ---------------------------------------------------------------------------

void NavigationCoordinatorNode::nav_goal_response(NavToPoseHandle::SharedPtr handle) {
    if (!handle) {
        publish_navigation_event(
            NavigationEvent::EVENT_GOAL_REJECTED, "go-to-pose goal rejected",
            current_pose_, 0.0f, builtin_interfaces::msg::Duration(), 0, 0,
            mission_.total_waypoints);
        publish_status(MissionState::ABORTED, "goal rejected by server");
        finish_mission(NavigationEvent::EVENT_GOAL_REJECTED, false, "goal rejected by server");
        return;
    }
    active_nav_handle_ = handle;
    publish_navigation_event(
        NavigationEvent::EVENT_GOAL_ACCEPTED, "go-to-pose goal accepted",
        current_pose_, 0.0f, builtin_interfaces::msg::Duration(), 0, 0,
        mission_.total_waypoints);
    publish_status(MissionState::NAVIGATING, "goal accepted");
}

void NavigationCoordinatorNode::nav_feedback(
    NavToPoseHandle::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    publish_navigation_event(
        NavigationEvent::EVENT_FEEDBACK, "feedback", feedback->current_pose,
        feedback->distance_remaining, feedback->estimated_time_remaining,
        feedback->number_of_recoveries, 0, mission_.total_waypoints);
    send_action_feedback("executing go-to-pose");
}

void NavigationCoordinatorNode::nav_result(const NavToPoseHandle::WrappedResult & result) {
    active_nav_handle_.reset();

    if (cancel_reason_ == CancelReason::PAUSE) {
        cancel_reason_ = CancelReason::NONE;
        state_ = MissionState::PAUSED;
        publish_status(MissionState::PAUSED, "mission paused");
        return;
    }
    if (cancel_reason_ == CancelReason::REPLACE) {
        cancel_reason_ = CancelReason::NONE;
        auto msg = pending_mission_;
        pending_mission_ = nullptr;
        state_ = MissionState::IDLE;
        if (msg) {
            start_mission(msg);
        }
        return;
    }

    cancel_reason_ = CancelReason::NONE;
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        publish_status(MissionState::COMPLETED, "mission completed");
        finish_mission(NavigationEvent::EVENT_GOAL_COMPLETED, true, "mission completed");
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
        std::string reason = "mission aborted";
        if (result.result) {
            reason += ": " + result.result->error_msg;
        }
        publish_status(MissionState::ABORTED, reason);
        finish_mission(NavigationEvent::EVENT_GOAL_ABORTED, false, reason);
    } else {  // CANCELED
        publish_status(MissionState::CANCELED, "mission canceled");
        finish_mission(NavigationEvent::EVENT_GOAL_CANCELED, false, "mission canceled");
    }
}

void NavigationCoordinatorNode::follow_goal_response(FollowWaypointsHandle::SharedPtr handle) {
    if (!handle) {
        publish_navigation_event(
            NavigationEvent::EVENT_GOAL_REJECTED, "waypoint goal rejected",
            current_pose_, 0.0f, builtin_interfaces::msg::Duration(), 0, 0,
            mission_.total_waypoints);
        publish_status(MissionState::ABORTED, "waypoint goal rejected by server");
        finish_mission(NavigationEvent::EVENT_GOAL_REJECTED, false, "waypoint goal rejected by server");
        return;
    }
    active_follow_handle_ = handle;
    publish_navigation_event(
        NavigationEvent::EVENT_GOAL_ACCEPTED, "waypoint goal accepted",
        current_pose_, 0.0f, builtin_interfaces::msg::Duration(), 0, 0,
        mission_.total_waypoints);
    publish_status(MissionState::NAVIGATING, "waypoint goal accepted");
}

void NavigationCoordinatorNode::follow_feedback(
    FollowWaypointsHandle::SharedPtr,
    const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
    mission_.current_waypoint = feedback->current_waypoint;
    builtin_interfaces::msg::Duration eta;
    float distance = distance_remaining_for_waypoints();
    float eta_sec = (average_speed_ > 0.0) ? distance / average_speed_ : 0.0f;
    eta.sec = static_cast<int32_t>(eta_sec);
    eta.nanosec = static_cast<uint32_t>((eta_sec - static_cast<int32_t>(eta_sec)) * 1e9);

    publish_navigation_event(
        NavigationEvent::EVENT_FEEDBACK, "waypoint feedback", current_pose_,
        distance, eta, mission_current_recoveries_, feedback->current_waypoint,
        mission_.total_waypoints);
    send_action_feedback("executing waypoints");
}

void NavigationCoordinatorNode::follow_result(const FollowWaypointsHandle::WrappedResult & result) {
    std::vector<uint32_t> missed;
    for (const auto & wp : result.result->missed_waypoints) {
        missed.push_back(wp.index);
    }
    active_follow_handle_.reset();

    if (cancel_reason_ == CancelReason::PAUSE) {
        cancel_reason_ = CancelReason::NONE;
        state_ = MissionState::PAUSED;
        // preserve remaining waypoints from current waypoint onwards
        std::vector<geometry_msgs::msg::PoseStamped> remaining;
        for (size_t i = mission_.current_waypoint; i < mission_.waypoints.size(); ++i) {
            remaining.push_back(mission_.waypoints[i]);
        }
        mission_.waypoints = remaining;
        mission_.remaining_waypoints = remaining;
        mission_.current_waypoint = 0;
        mission_.total_waypoints = mission_.waypoints.size();
        publish_waypoint_markers();
        publish_status(MissionState::PAUSED, "mission paused at waypoint");
        return;
    }
    if (cancel_reason_ == CancelReason::REPLACE) {
        cancel_reason_ = CancelReason::NONE;
        auto msg = pending_mission_;
        pending_mission_ = nullptr;
        state_ = MissionState::IDLE;
        if (msg) {
            start_mission(msg);
        }
        return;
    }

    cancel_reason_ = CancelReason::NONE;
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        publish_status(MissionState::COMPLETED, "waypoint mission completed");
        finish_mission(NavigationEvent::EVENT_GOAL_COMPLETED, true, "waypoint mission completed");
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
        publish_status(MissionState::ABORTED,
                       "waypoint mission aborted (" + result.result->error_msg + ")");
        finish_mission(NavigationEvent::EVENT_GOAL_ABORTED, false, "waypoint mission aborted");
    } else {  // CANCELED
        publish_status(MissionState::CANCELED, "waypoint mission canceled");
        finish_mission(NavigationEvent::EVENT_GOAL_CANCELED, false, "waypoint mission canceled");
    }
}

// ---------------------------------------------------------------------------
// ExecuteMission action server
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse NavigationCoordinatorNode::mission_goal_callback(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteMission::Goal> goal) {
    (void)uuid;
    if (state_ == MissionState::NAVIGATING || state_ == MissionState::PAUSED) {
        RCLCPP_WARN(this->get_logger(), "ExecuteMission rejected: mission already active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void NavigationCoordinatorNode::mission_accepted_callback(
    const std::shared_ptr<MissionServerGoalHandle> handle) {
    active_mission_goal_ = handle;
    std::thread(
        std::bind(&NavigationCoordinatorNode::mission_execute_callback, this,
                  handle))
        .detach();
}

rclcpp_action::CancelResponse NavigationCoordinatorNode::mission_cancel_callback(
    const std::shared_ptr<MissionServerGoalHandle> handle) {
    (void)handle;
    RCLCPP_INFO(this->get_logger(), "ExecuteMission goal canceled");
    handle_cancel();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigationCoordinatorNode::mission_execute_callback(
    const std::shared_ptr<MissionServerGoalHandle> handle) {
    if (handle->is_canceling()) {
        handle_cancel();
        auto result = std::make_shared<ExecuteMission::Result>();
        result->success = false;
        result->message = "mission canceled";
        handle->canceled(result);
        return;
    }

    const auto goal = handle->get_goal();

    NavigationMission::SharedPtr msg = std::make_shared<NavigationMission>();
    msg->header.stamp = this->now();
    msg->mission_id = goal->mission_id;
    msg->mode = goal->mode;
    msg->target_pose = goal->target_pose;
    msg->waypoints = goal->waypoints;

    start_mission(msg);
}

void NavigationCoordinatorNode::send_action_feedback(const std::string & status) {
    if (!active_mission_goal_ || !active_mission_goal_->is_active()) {
        return;
    }
    auto feedback = std::make_shared<ExecuteMission::Feedback>();
    feedback->current_pose = current_pose_;
    feedback->distance_remaining = distance_remaining_for_waypoints();
    float eta_sec = (average_speed_ > 0.0)
                            ? feedback->distance_remaining / average_speed_
                            : 0.0f;
    feedback->estimated_time_remaining.sec = static_cast<int32_t>(eta_sec);
    feedback->estimated_time_remaining.nanosec =
        static_cast<uint32_t>((eta_sec - static_cast<int32_t>(eta_sec)) * 1e9);
    feedback->number_of_recoveries = mission_current_recoveries_;
    feedback->current_waypoint = mission_.current_waypoint;
    feedback->total_waypoints = mission_.total_waypoints;
    feedback->status = status;
    active_mission_goal_->publish_feedback(feedback);
}

void NavigationCoordinatorNode::finish_mission(
    uint8_t result_event, bool success, const std::string & message) {
    publish_navigation_event(
        result_event, message, current_pose_, 0.0f,
        builtin_interfaces::msg::Duration(), 0, 0, mission_.total_waypoints);

    if (active_mission_goal_ && active_mission_goal_->is_active()) {
        auto result = std::make_shared<ExecuteMission::Result>();
        result->success = success;
        result->message = message;
        if (result_event == NavigationEvent::EVENT_GOAL_CANCELED) {
            active_mission_goal_->canceled(result);
        } else if (!success) {
            active_mission_goal_->abort(result);
        } else {
            active_mission_goal_->succeed(result);
        }
        active_mission_goal_.reset();
    }
    state_ = MissionState::IDLE;
    mission_ = MissionContext();
    publish_waypoint_markers();
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void NavigationCoordinatorNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
    have_pose_ = true;
}

float NavigationCoordinatorNode::distance_remaining_for_waypoints() {
    const auto & wps = mission_.waypoints;
    if (wps.empty()) {
        return 0.0f;
    }

    double total = 0.0;
    double cx = have_pose_ ? current_pose_.pose.position.x : 0.0;
    double cy = have_pose_ ? current_pose_.pose.position.y : 0.0;
    // clamp index into list
    size_t start = std::min<size_t>(mission_.current_waypoint, wps.size() - 1);

    for (size_t i = start; i < wps.size(); ++i) {
        double tx = wps[i].pose.position.x;
        double ty = wps[i].pose.position.y;
        total += (i == start)
                     ? std::hypot(tx - cx, ty - cy)
                     : std::hypot(tx - wps[i - 1].pose.position.x,
                                  ty - wps[i - 1].pose.position.y);
    }
    return static_cast<float>(total);
}

void NavigationCoordinatorNode::publish_navigation_event(
    uint8_t event_type, const std::string & message,
    const geometry_msgs::msg::PoseStamped & pose, float distance,
    builtin_interfaces::msg::Duration eta, int16_t recoveries,
    uint32_t current_wp, uint32_t total_wp) {
    NavigationEvent evt;
    evt.header.stamp = this->now();
    evt.mission_id = mission_.mission_id;
    evt.event_type = event_type;
    evt.message = message;
    evt.current_pose = pose;
    evt.distance_remaining = distance;
    evt.estimated_time_remaining = eta;
    evt.number_of_recoveries = recoveries;
    evt.current_waypoint = current_wp;
    evt.total_waypoints = total_wp;
    event_pub_->publish(evt);
}

void NavigationCoordinatorNode::publish_status(MissionState state, const std::string & message) {
    NavigationStatus status;
    status.header.stamp = this->now();
    status.mission_id = mission_.mission_id;
    status.current_pose = current_pose_;
    status.message = message;
    status.total_waypoints = mission_.total_waypoints;
    status.current_waypoint = mission_.current_waypoint;
    switch (state) {
        case MissionState::IDLE: status.state = NavigationStatus::STATE_IDLE; break;
        case MissionState::NAVIGATING: status.state = NavigationStatus::STATE_NAVIGATING; break;
        case MissionState::PAUSED: status.state = NavigationStatus::STATE_PAUSED; break;
        case MissionState::COMPLETED: status.state = NavigationStatus::STATE_COMPLETED; break;
        case MissionState::CANCELED: status.state = NavigationStatus::STATE_CANCELED; break;
        case MissionState::ABORTED: status.state = NavigationStatus::STATE_ABORTED; break;
    }
    status_pub_->publish(status);
}

void NavigationCoordinatorNode::publish_waypoint_markers() {
    visualization_msgs::msg::MarkerArray markers;

    // Clear-all marker so stale waypoints disappear when a mission ends.
    visualization_msgs::msg::Marker clear;
    clear.header.stamp = this->now();
    clear.header.frame_id = "map";
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    std::vector<geometry_msgs::msg::PoseStamped> points;
    if (mission_.mode == NavigationMission::MODE_GO_TO_POSE) {
        if (!mission_.mission_id.empty()) {
            points.push_back(mission_.target_pose);
        }
    } else {
        points = mission_.waypoints;
    }

    for (size_t i = 0; i < points.size(); ++i) {
        const auto & p = points[i];

        visualization_msgs::msg::Marker sphere;
        sphere.header = p.header;
        sphere.header.stamp = this->now();
        sphere.header.frame_id = p.header.frame_id.empty() ? "map" : p.header.frame_id;
        sphere.ns = "mission_waypoints";
        sphere.id = static_cast<int>(i);
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose = p.pose;
        sphere.pose.position.z = 0.05;
        sphere.scale.x = 0.15;
        sphere.scale.y = 0.15;
        sphere.scale.z = 0.15;
        sphere.color.r = 0.1f;
        sphere.color.g = 0.9f;
        sphere.color.b = 0.1f;
        sphere.color.a = 0.9f;
        markers.markers.push_back(sphere);

        visualization_msgs::msg::Marker label;
        label.header = sphere.header;
        label.ns = "mission_waypoint_labels";
        label.id = static_cast<int>(i);
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        label.pose = p.pose;
        label.pose.position.z = 0.4;
        label.scale.z = 0.25;
        label.color.r = 1.0f;
        label.color.g = 1.0f;
        label.color.b = 1.0f;
        label.color.a = 1.0f;
        label.text = std::to_string(i);
        markers.markers.push_back(label);
    }

    waypoint_marker_pub_->publish(markers);
}

} // namespace ms04_autonomous_navigation

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ms04_autonomous_navigation::NavigationCoordinatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}