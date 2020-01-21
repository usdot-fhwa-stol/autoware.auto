// Copyright 2020 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/// \file
/// \brief Base class for vehicle drivers
#ifndef VEHICLE_INTERFACE__VEHICLE_INTERFACE_NODE_HPP_
#define VEHICLE_INTERFACE__VEHICLE_INTERFACE_NODE_HPP_

#include <vehicle_interface/platform_interface.hpp>
#include <vehicle_interface/safety_state_machine.hpp>
#include <vehicle_interface/visibility_control.hpp>

#include <rclcpp/rclcpp.hpp>
#include <reference_tracking_controller/reference_tracking_controller.hpp>
#include <signal_filters/signal_filter.hpp>

#include <autoware_auto_msgs/msg/high_level_control_command.hpp>
#include <autoware_auto_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_odometry.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>

#include <optional>
#include <variant>
#include <chrono>
#include <exception>
#include <memory>
#include <string>

namespace autoware
{
namespace drivers
{
namespace vehicle_interface
{
using Real = decltype(BasicControlCommand::long_accel_mps2);

/// Convenience struct for construction
struct TopicNumMatches
{
  std::string topic;
};  // struct TopicNumMatches

/// Convenience struct to construct filters
struct FilterConfig
{
  std::string type;
  Real cutoff_frequency;
};  // struct FilterConfig

/// A node which receives commands and sends them to the vehicle platform, and publishes
/// reports from the vehicle platform
class VEHICLE_INTERFACE_PUBLIC VehicleInterfaceNode : public ::rclcpp::Node
{
public:
  /// ROS 2 parameter constructor
  /// \param[in] node_name The name of the node
  /// \param[in] node_namespace Namespace of the node
  VehicleInterfaceNode(
    const std::string & node_name,
    const std::string & node_namespace = "");

  /// Explicit parameter constructor
  /// \param[in] node_name The name of the node
  /// \param[in] node_namespace Namespace of the node
  /// \param[in] cycle_time Duration to exchange data with vehicle platform
  /// \param[in] raw_command Topic name and expected number of matches for raw control command
  /// \param[in] basic_command Topic name and expected number of matches for basic control command
  /// \param[in] high_level_command Topic name and expected number of matches for high level control
  ///                               command
  /// \param[in] state_command Topic name and expected number of matches for state command
  /// \param[in] odometry Topic name and expected number of matches for odometry
  /// \param[in] state_report Topic name and expected number of matches for state report command
  /// \param[in] state_machine_config Configuration class for safety state machine
  /// \param[in] longitudinal_filter Filter for acceleration/velocity for all commands
  /// \param[in] curvature_filter Filter for curvature component of high level control commands
  /// \param[in] front_steer_filter Filter for front steer component in raw or basic commands
  /// \param[in] rear_steer_filter Filter for rear steer component in raw or basic commands
  VehicleInterfaceNode(
    const std::string & node_name,
    const std::string & node_namespace,
    const std::chrono::nanoseconds cycle_time,
    const TopicNumMatches & raw_command,
    const TopicNumMatches & basic_command,
    const TopicNumMatches & high_level_command,
    const TopicNumMatches & state_command,
    const TopicNumMatches & odometry,
    const TopicNumMatches & state_report,
    const std::optional<StateMachineConfig> & state_machine_config,
    const FilterConfig & longitudinal_filter,
    const FilterConfig & curvature_filter,
    const FilterConfig & front_steer_filter,
    const FilterConfig & rear_steer_filter);

protected:
  using ControllerBasePtr =
    std::unique_ptr<common::reference_tracking_controller::ReferenceTrackerBase<Real>>;
  using FilterBasePtr = std::unique_ptr<common::signal_filters::FilterBase<Real>>;
  struct VehicleFilter
  {
    FilterBasePtr longitudinal;
    FilterBasePtr curvature;
    FilterBasePtr front_steer;
    FilterBasePtr rear_steer;
  };
  /// Set the low pass filter
  void set_filter(VehicleFilter && filter) noexcept;
  /// Set the reference tracker (controller)
  void set_reference_tracker(ControllerBasePtr && controller) noexcept;
  /// Set the vehicle-specific PlatformInterface
  void set_interface(std::unique_ptr<PlatformInterface> && interface) noexcept;
  /// Get access to logger
  rclcpp::Logger logger() const noexcept;

  /// Error handling behavior for when sending a control command has failed, default is throwing an
  /// exception, which is caught and turned into a change in the NodeState to ERROR
  /// TODO(c.ho) add command which failed to send as an argument
  virtual void on_control_send_failure();
  /// Error handling behavior for when sending a state command has failed, default is throwing an
  /// exception, which is caught and turned into a change in the NodeState to ERROR
  /// TODO(c.ho) add command which failed to send as an argument
  virtual void on_state_send_failure();
  /// Error handling behavior for when receiving data from the vehicle platform has timed out,
  /// default is throwing an exception, which is caught and turned into a change in the NodeState to
  /// ERROR
  virtual void on_read_timeout();
  /// Handle exception thrown in main loop. Default behavior is to set NodeState to ERROR
  virtual void on_error(std::exception_ptr eptr);

private:
  // Helper function called in constructors
  VEHICLE_INTERFACE_LOCAL void init(
    const TopicNumMatches & raw_command,
    const TopicNumMatches & basic_command,
    const TopicNumMatches & high_level_command,
    const TopicNumMatches & state_command,
    const TopicNumMatches & odometry,
    const TopicNumMatches & state_report,
    const std::optional<StateMachineConfig> & state_machine_config,
    const FilterConfig & longitudinal_filter,
    const FilterConfig & curvature_filter,
    const FilterConfig & front_steer_filter,
    const FilterConfig & rear_steer_filter,
    const std::chrono::nanoseconds & cycle_time);

  // Run just before main loop, ensure that all invariants (possibly from child class) are enforced
  VEHICLE_INTERFACE_LOCAL void check_invariants();

  // Send state command
  VEHICLE_INTERFACE_LOCAL void send_state_command(const MaybeStateCommand & maybe_command);
  // Read data from vehicle platform for time budget, publish data
  VEHICLE_INTERFACE_LOCAL void read_and_publish();
  // Core loop for different input commands. Specialized differently for each topic type
  template<typename T>
  VEHICLE_INTERFACE_LOCAL void on_command_message(const T & msg);
  /// Log a warning from the safety state machine: transition node state and/or log
  VEHICLE_INTERFACE_LOCAL void state_machine_report();

  rclcpp::TimerBase::SharedPtr m_read_timer{nullptr};
  rclcpp::Publisher<autoware_auto_msgs::msg::VehicleOdometry>::SharedPtr m_odom_pub{nullptr};
  rclcpp::Publisher<autoware_auto_msgs::msg::VehicleStateReport>::SharedPtr m_state_pub{nullptr};
  rclcpp::Subscription<autoware_auto_msgs::msg::VehicleStateCommand>::SharedPtr m_state_sub{};

  using BasicSub = rclcpp::Subscription<BasicControlCommand>::SharedPtr;
  using RawSub = rclcpp::Subscription<autoware_auto_msgs::msg::RawControlCommand>::SharedPtr;
  using HighLevelSub =
    rclcpp::Subscription<autoware_auto_msgs::msg::HighLevelControlCommand>::SharedPtr;

  std::variant<RawSub, BasicSub, HighLevelSub> m_command_sub{};

  std::unique_ptr<PlatformInterface> m_interface{nullptr};
  VehicleFilter m_filter{nullptr, nullptr, nullptr, nullptr};
  ControllerBasePtr m_controller{nullptr};
  std::unique_ptr<SafetyStateMachine> m_state_machine{nullptr};
  std::chrono::system_clock::time_point m_last_command_stamp{};
  std::chrono::nanoseconds m_cycle_time{};
  MaybeStateCommand m_last_state_command{};
};  // class VehicleInterfaceNode

}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace autoware

#endif  // VEHICLE_INTERFACE__VEHICLE_INTERFACE_NODE_HPP_
