// Copyright 2020 Embotech AG, Zurich, Switzerland, inspired by Christopher Ho's mpc code
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <common/types.hpp>
#include <recordreplay_planner_nodes/recordreplay_planner_node.hpp>
#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <memory>
#include <string>
#include <utility>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;

namespace motion
{
namespace planning
{
namespace recordreplay_planner_nodes
{
RecordReplayPlannerNode::RecordReplayPlannerNode(const rclcpp::NodeOptions & node_options)
: Node{"recordreplay_planner", node_options}
{
  const auto ego_topic = "vehicle_state";
  const auto trajectory_topic = "planned_trajectory";
  const auto trajectory_viz_topic = "planned_trajectory_viz";
  const auto heading_weight = declare_parameter("heading_weight", rclcpp::PARAMETER_DOUBLE).get<float64_t>();
  const auto min_record_distance = declare_parameter("min_record_distance", rclcpp::PARAMETER_DOUBLE).get<float64_t>();
  m_goal_distance_threshold_m = declare_parameter("goal_distance_threshold_m", rclcpp::PARAMETER_DOUBLE).get<float32_t>();
  m_goal_angle_threshold_rad = declare_parameter("goal_angle_threshold_rad", rclcpp::PARAMETER_DOUBLE).get<float32_t>();

  using rclcpp::QoS;
  using namespace std::chrono_literals;

  // Setup Tf Buffer with listener
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_,
    std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false);

  // Set up action for control of recording and replaying
  m_recordserver = rclcpp_action::create_server<RecordTrajectory>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "recordtrajectory",
    [this](auto uuid, auto goal) {return this->record_handle_goal(uuid, goal);},
    [this](auto goal_handle) {return this->record_handle_cancel(goal_handle);},
    [this](auto goal_handle) {return this->record_handle_accepted(goal_handle);});

  m_replayserver = rclcpp_action::create_server<ReplayTrajectory>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "replaytrajectory",
    [this](auto uuid, auto goal) {return this->replay_handle_goal(uuid, goal);},
    [this](auto goal_handle) {return this->replay_handle_cancel(goal_handle);},
    [this](auto goal_handle) {return this->replay_handle_accepted(goal_handle);});

  // Set up subscribers for the actual recording
  using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
  m_ego_sub = create_subscription<State>(
    ego_topic, QoS{10},
    [this](const State::SharedPtr msg) {on_ego(msg);}, SubAllocT{});

  // Set up publishers
  using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
  m_trajectory_pub =
    create_publisher<Trajectory>(trajectory_topic, QoS{10}, PubAllocT{});
  m_trajectory_viz_pub =
    create_publisher<MarkerArray>(trajectory_viz_topic, QoS{10});

  // Set up services
  if (declare_parameter("enable_object_collision_estimator", rclcpp::PARAMETER_BOOL).get<bool>()) {
    m_modify_trajectory_client = this->create_client<ModifyTrajectory>("estimate_collision");
    while (!m_modify_trajectory_client->wait_for_service(3s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(get_logger(), "Waiting for service connection...");
    }
  }

  // Create and set a planner object that we'll talk to
  m_planner = std::make_unique<recordreplay_planner::RecordReplayPlanner>();
  m_planner->set_heading_weight(heading_weight);
  m_planner->set_min_record_distance(min_record_distance);
}

Marker RecordReplayPlannerNode::to_marker(
  const TrajectoryPoint & traj_point,
  const std::string & frame_id,
  int32_t index,
  const std::string & ns)
{
  Marker marker;

  tf2::Quaternion quat;
  quat.setEuler(0.0, 0.0, motion::motion_common::to_angle(traj_point.heading));

  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Time(0);
  marker.ns = ns;
  marker.id = index;
  marker.type = Marker::ARROW;
  marker.action = Marker::ADD;
  marker.pose.position.x = traj_point.x;
  marker.pose.position.y = traj_point.y;
  marker.pose.orientation = toMsg(quat);
  marker.scale.x = 0.5;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;

  if (m_planner->is_recording()) {
    marker.color.r = 0.75f;
    marker.color.g = 0.0f;
    marker.color.b = 0.75f;
    marker.color.a = 1.0f;
  } else if (m_planner->is_replaying()) {
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.75f;
    marker.color.a = 1.0f;
  }

  marker.frame_locked = false;

  return marker;
}

MarkerArray RecordReplayPlannerNode::to_markers(const Trajectory & traj, const std::string & ns)
{
  visualization_msgs::msg::MarkerArray markers;
  int32_t index = 0;

  for (const auto & traj_point : traj.points) {
    markers.markers.push_back(to_marker(traj_point, traj.header.frame_id, index, ns));
    index++;
  }

  return markers;
}

void RecordReplayPlannerNode::clear_recorded_markers()
{
  if (m_recorded_markers.markers.size() > 0) {
    for (auto & marker : m_recorded_markers.markers) {
      marker.action = Marker::DELETE;
    }
    m_trajectory_viz_pub->publish(m_recorded_markers);
  }

  m_recorded_markers.markers.clear();
}

void RecordReplayPlannerNode::on_ego(const State::SharedPtr & msg)
{
  if (m_odom_frame_id.empty()) {
    m_odom_frame_id = msg->header.frame_id;
  }

  if (m_planner->is_recording()) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Recording ego position");
    const auto added = m_planner->record_state(*msg);

    if (added) {
      // Publish visualization markers
      m_recorded_markers.markers.push_back(
        to_marker(
          msg->state,
          msg->header.frame_id,
          static_cast<int>(m_planner->get_record_length()),
          "record"));
      m_trajectory_viz_pub->publish(m_recorded_markers);
    }

    // Publish recording feedback information
    auto feedback_msg = std::make_shared<RecordTrajectory::Feedback>();
    feedback_msg->current_length = static_cast<int32_t>(m_planner->get_record_length());
    m_recordgoalhandle->publish_feedback(feedback_msg);
  }

  if (m_planner->is_replaying()) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Replaying recorded ego postion as trajectory");
    const auto & traj_raw = m_planner->plan(*msg);

    // Request service to consider object collision if enabled
    if (m_modify_trajectory_client) {
      auto request = std::make_shared<ModifyTrajectory::Request>();
      request->original_trajectory = traj_raw;
      auto result = m_modify_trajectory_client->async_send_request(
        request, std::bind(
          &RecordReplayPlannerNode::modify_trajectory_response,
          this, std::placeholders::_1));
    } else {
      m_trajectory_pub->publish(traj_raw);

      // Publish visualization markers
      auto markers = to_markers(traj_raw, "replay");
      m_trajectory_viz_pub->publish(markers);

      // Publish replaying feedback information
      auto feedback_msg = std::make_shared<ReplayTrajectory::Feedback>();
      feedback_msg->remaining_length = static_cast<int32_t>(traj_raw.points.size());
      m_replaygoalhandle->publish_feedback(feedback_msg);
    }

    if (m_planner->reached_goal(*msg, m_goal_distance_threshold_m, m_goal_angle_threshold_rad)) {
      m_replaygoalhandle->succeed(std::make_shared<ReplayTrajectory::Result>());
      m_planner->stop_replaying();
    }
  }
}

void RecordReplayPlannerNode::modify_trajectory_response(
  rclcpp::Client<ModifyTrajectory>::SharedFuture future)
{
  auto traj = future.get()->modified_trajectory;
  m_trajectory_pub->publish(traj);

  // Publish visualization markers
  auto markers = to_markers(traj, "replay");
  m_trajectory_viz_pub->publish(markers);

  // Publish replaying feedback information
  auto feedback_msg = std::make_shared<ReplayTrajectory::Feedback>();
  feedback_msg->remaining_length = static_cast<int32_t>(traj.points.size());
  m_replaygoalhandle->publish_feedback(feedback_msg);
}

rclcpp_action::GoalResponse RecordReplayPlannerNode::record_handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  const std::shared_ptr<const RecordTrajectory::Goal> goal)
{
  (void)goal;
  (void)uuid;
  if (m_planner->is_recording()) {
    // Can't start recording if we already are
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RecordReplayPlannerNode::record_handle_cancel(
  const std::shared_ptr<GoalHandleRecordTrajectory> goal_handle)
{
  if (m_planner->is_recording()) {
    RCLCPP_INFO(this->get_logger(), "Cancel recording");
    m_planner->stop_recording();

    std::string record_path = goal_handle->get_goal()->record_path;

    // If a path is specified
    if (record_path.length() > 0) {
      // Write trajectory to file
      m_planner->writeTrajectoryBufferToFile(
        goal_handle->get_goal()->record_path);
    }
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RecordReplayPlannerNode::record_handle_accepted(
  const std::shared_ptr<GoalHandleRecordTrajectory> goal_handle)
{
  // Store the goal handle otherwise the action gets canceled immediately
  m_recordgoalhandle = goal_handle;
  m_planner->start_recording();

  // If a path was recorded previously, clear the markers
  clear_recorded_markers();
}

rclcpp_action::GoalResponse RecordReplayPlannerNode::replay_handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  const std::shared_ptr<const ReplayTrajectory::Goal> goal)
{
  (void)goal;
  (void)uuid;
  if (m_planner->is_replaying()) {
    // Can't start replaying if we already are
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RecordReplayPlannerNode::replay_handle_cancel(
  const std::shared_ptr<GoalHandleReplayTrajectory> goal_handle)
{
  (void)goal_handle;
  if (m_planner->is_replaying()) {
    RCLCPP_INFO(this->get_logger(), "Cancel replaying");
    m_planner->stop_replaying();
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RecordReplayPlannerNode::replay_handle_accepted(
  const std::shared_ptr<GoalHandleReplayTrajectory> goal_handle)
{
  // Store the goal handle otherwise the action gets canceled immediately
  m_replaygoalhandle = goal_handle;

  std::string replay_path = goal_handle->get_goal()->replay_path;

  // If a path is specified
  if (replay_path.length() > 0) {
    // Read trajectory from file
    m_planner->readTrajectoryBufferFromFile(
      goal_handle->get_goal()->replay_path);
  }

  // If a path was recorded previously, clear the markers
  clear_recorded_markers();

  // Publish loaded states as replaying feedback information
  auto feedback_msg = std::make_shared<ReplayTrajectory::Feedback>();
  auto & remaining_length = feedback_msg->remaining_length;
  remaining_length = static_cast<int32_t>(m_planner->get_record_length());
  m_replaygoalhandle->publish_feedback(feedback_msg);

  // Start the replaying process
  m_planner->start_replaying();
}
}  // namespace recordreplay_planner_nodes
}  // namespace planning
}  // namespace motion

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(
  motion::planning::recordreplay_planner_nodes::RecordReplayPlannerNode)
