#ifndef MS04_AUTONOMOUS_NAVIGATION__NAVIGATION_COORDINATOR_NODE_HPP_
#define MS04_AUTONOMOUS_NAVIGATION__NAVIGATION_COORDINATOR_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "ms04_autonomous_navigation/action/execute_mission.hpp"
#include "ms04_autonomous_navigation/msg/navigation_event.hpp"
#include "ms04_autonomous_navigation/msg/navigation_mission.hpp"
#include "ms04_autonomous_navigation/msg/navigation_status.hpp"

namespace ms04_autonomous_navigation {

/**
 * @class NavigationCoordinatorNode
 * @brief Unified Nav2 coordinator node.
 *
 * Acts as an action client to Nav2's NavigateToPose and FollowWaypoints,
 * dispatches unified NavigationMission commands (single-pose goals and
 * multi-waypoint routes), implements Start/Cancel/Pause/Resume/Replace
 * control, logs all events to /navigation/events and exposes an
 * ExecuteMission action server plus a /navigation/status stream.
 */
class NavigationCoordinatorNode : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using ExecuteMission = ms04_autonomous_navigation::action::ExecuteMission;
    using NavigationMission = ms04_autonomous_navigation::msg::NavigationMission;
    using NavigationEvent = ms04_autonomous_navigation::msg::NavigationEvent;
    using NavigationStatus = ms04_autonomous_navigation::msg::NavigationStatus;

    using NavigateToPoseClient = rclcpp_action::Client<NavigateToPose>;
    using FollowWaypointsClient = rclcpp_action::Client<FollowWaypoints>;
    using ExecuteMissionServer = rclcpp_action::Server<ExecuteMission>;

    using NavToPoseHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using FollowWaypointsHandle = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
    using MissionServerGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteMission>;

    explicit NavigationCoordinatorNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    enum class MissionState {
        IDLE,
        NAVIGATING,
        PAUSED,
        COMPLETED,
        CANCELED,
        ABORTED,
    };

    enum class CancelReason {
        NONE,
        USER_CANCEL,
        PAUSE,
        REPLACE,
    };

    struct MissionContext {
        std::string mission_id;
        uint8_t mode;
        geometry_msgs::msg::PoseStamped target_pose;
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        std::vector<geometry_msgs::msg::PoseStamped> remaining_waypoints;
        uint32_t current_waypoint;
        uint32_t total_waypoints;
        float distance_remaining;
    };

    // ---- Command dispatcher ----
    void mission_command_callback(const NavigationMission::SharedPtr msg);
    void process_command(
        uint8_t command, uint8_t mode, const NavigationMission::SharedPtr msg);
    void start_mission(const NavigationMission::SharedPtr msg);
    void replace_mission(const NavigationMission::SharedPtr msg);
    void dispatch_mission(const MissionContext & ctx);
    void dispatch_navigate_to_pose(const geometry_msgs::msg::PoseStamped & pose);
    void dispatch_follow_waypoints(const std::vector<geometry_msgs::msg::PoseStamped> & poses);

    // ---- Control handlers ----
    void handle_cancel();
    void handle_pause();
    void handle_resume();
    void handle_replace(const NavigationMission::SharedPtr msg);

    // ---- Nav2 NavigateToPose callbacks ----
    void nav_goal_response(NavToPoseHandle::SharedPtr handle);
    void nav_feedback(
        NavToPoseHandle::SharedPtr handle,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void nav_result(const NavToPoseHandle::WrappedResult & result);

    // ---- Nav2 FollowWaypoints callbacks ----
    void follow_goal_response(FollowWaypointsHandle::SharedPtr handle);
    void follow_feedback(
        FollowWaypointsHandle::SharedPtr handle,
        const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
    void follow_result(const FollowWaypointsHandle::WrappedResult & result);

    // ---- ExecuteMission action server callbacks ----
    rclcpp_action::GoalResponse mission_goal_callback(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ExecuteMission::Goal> goal);
    void mission_accepted_callback(const std::shared_ptr<MissionServerGoalHandle> handle);
    rclcpp_action::CancelResponse mission_cancel_callback(
        const std::shared_ptr<MissionServerGoalHandle> handle);
    void mission_execute_callback(const std::shared_ptr<MissionServerGoalHandle> handle);

    // ---- Helpers ----
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    float distance_remaining_for_waypoints();
    void publish_navigation_event(
        uint8_t event_type, const std::string & message,
        const geometry_msgs::msg::PoseStamped & pose, float distance,
        builtin_interfaces::msg::Duration eta, int16_t recoveries,
        uint32_t current_wp, uint32_t total_wp);
    void publish_status(MissionState state, const std::string & message);
    void send_action_feedback(const std::string & status);
    void finish_mission(
        uint8_t result_event, bool success, const std::string & message);

    // ---- members ----
    std::string navigate_to_pose_action_;
    std::string follow_waypoints_action_;
    std::string execute_mission_action_;

    NavigateToPoseClient::SharedPtr navigate_to_pose_client_;
    FollowWaypointsClient::SharedPtr follow_waypoints_client_;
    ExecuteMissionServer::SharedPtr execute_mission_server_;

    NavToPoseHandle::SharedPtr active_nav_handle_;
    FollowWaypointsHandle::SharedPtr active_follow_handle_;
    std::shared_ptr<MissionServerGoalHandle> active_mission_goal_;

    rclcpp::Subscription<NavigationMission>::SharedPtr mission_sub_;
    rclcpp::Publisher<NavigationEvent>::SharedPtr event_pub_;
    rclcpp::Publisher<NavigationStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    MissionContext mission_;
    MissionState state_;
    CancelReason cancel_reason_;
    NavigationMission::SharedPtr pending_mission_;
    int16_t mission_current_recoveries_;

    geometry_msgs::msg::PoseStamped current_pose_;
    bool have_pose_;

    double average_speed_;
};

} // namespace ms04_autonomous_navigation

#endif // MS04_AUTONOMOUS_NAVIGATION__NAVIGATION_COORDINATOR_NODE_HPP_