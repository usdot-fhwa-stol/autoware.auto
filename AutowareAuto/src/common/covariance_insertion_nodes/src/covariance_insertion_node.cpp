// Copyright 2020 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2020 Apex.AI, Inc.
/// All rights reserved.

#include "covariance_insertion_nodes/covariance_insertion_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <string>
#include <memory>
#include <vector>
#include <map>

namespace autoware
{
namespace covariance_insertion_nodes
{

namespace
{
const std::int32_t kDefaultHistorySize = 10;
static constexpr auto kInputTopic = "messages";
static constexpr auto kOutputTopicSuffix = "_with_overriden_covariance";


static constexpr auto kHistorySizeTag = "history_size";
static constexpr auto kInputTopicTag = "input_topic";
static constexpr auto kInputMsgTypeTag = "input_msg_type";
static constexpr auto kOverrideCovariancesPrefix = "override_covariance";

static constexpr auto kOdometryMsgType = "Odometry";
static constexpr auto kPoseMsgType = "Pose";
static constexpr auto kPoseStampedMsgType = "PoseStamped";
static constexpr auto kPoseWithCovarianceMsgType = "PoseWithCovariance";
static constexpr auto kPoseWithCovarianceStampedMsgType = "PoseWithCovarianceStamped";
static constexpr auto kTwistMsgType = "Twist";
static constexpr auto kTwistStampedMsgType = "TwistStamped";
static constexpr auto kTwistWithCovarianceMsgType = "TwistWithCovariance";
static constexpr auto kTwistWithCovarianceStampedMsgType = "TwistWithCovarianceStamped";

static constexpr std::array<const char *, 3> kPossibleOverrideFieldNames{
  covariance_insertion::detail::kDirectlyTag, covariance_insertion::detail::kPoseTag,
  covariance_insertion::detail::kTwistTag};
}  // namespace

CovarianceInsertionNode::CovarianceInsertionNode(const rclcpp::NodeOptions & options)
:  Node("covariance_insertion_nodes", options)
{
  declare_parameter(kHistorySizeTag, kDefaultHistorySize);
  const auto history_size = get_parameter(kHistorySizeTag).as_int();
  if (history_size <= 0) {throw std::domain_error("History size must be positive.");}
  m_history_size = static_cast<std::size_t>(history_size);
  m_input_topic = kInputTopic;
  m_output_topic = m_input_topic + kOutputTopicSuffix;
  m_core = std::make_unique<covariance_insertion::CovarianceInsertion>();

  declare_parameter(kInputMsgTypeTag, rclcpp::PARAMETER_STRING);
  const auto input_msg_type_name = get_parameter(kInputMsgTypeTag).as_string();
  for (const auto & field : kPossibleOverrideFieldNames) {
    const auto full_field{std::string{kOverrideCovariancesPrefix} + '.' + field};
    declare_parameter(full_field, std::vector<common::types::float64_t>{});
    const auto covariance = get_parameter(full_field).as_double_array();
    if (!covariance.empty()) {
      m_core->insert_covariance(field, covariance);
    }
  }
  const auto msg_variant = fill_message_variant(input_msg_type_name);
  create_pub_sub(msg_variant);
}

CovarianceInsertionNode::MsgVariant CovarianceInsertionNode::fill_message_variant(
  const std::string & input_msg_type_name)
{
  if (input_msg_type_name == kOdometryMsgType) {
    return nav_msgs::msg::Odometry{};
  } else if (input_msg_type_name == kPoseMsgType) {
    return geometry_msgs::msg::Pose{};
  } else if (input_msg_type_name == kPoseStampedMsgType) {
    return geometry_msgs::msg::PoseStamped{};
  } else if (input_msg_type_name == kPoseWithCovarianceMsgType) {
    return geometry_msgs::msg::PoseWithCovariance{};
  } else if (input_msg_type_name == kPoseWithCovarianceStampedMsgType) {
    return geometry_msgs::msg::PoseWithCovarianceStamped{};
  } else if (input_msg_type_name == kTwistMsgType) {
    return geometry_msgs::msg::Twist{};
  } else if (input_msg_type_name == kTwistStampedMsgType) {
    return geometry_msgs::msg::TwistStamped{};
  } else if (input_msg_type_name == kTwistWithCovarianceMsgType) {
    return geometry_msgs::msg::TwistWithCovariance{};
  } else if (input_msg_type_name == kTwistWithCovarianceStampedMsgType) {
    return geometry_msgs::msg::TwistWithCovarianceStamped{};
  } else {
    throw std::runtime_error(
            "Unfortunately there is no implementation for message type: " + input_msg_type_name);
  }
}

void CovarianceInsertionNode::validate(const MsgVariant & msg_variant)
{
  if (m_core->covariances_empty()) {
    throw std::runtime_error(
            "No overrides provided. Probably something is wrong with the provided parameters.");
  }
  mpark::visit(
    [&](const auto & msg) {
      using InputMsgT = std::decay_t<decltype(msg)>;
      using OutputType = typename output<InputMsgT>::type;
      OutputType new_msg{};
      m_core->set_all_covariances(&new_msg);
    }, msg_variant);
}

void CovarianceInsertionNode::create_pub_sub(const MsgVariant & msg_variant)
{
  validate(msg_variant);
  mpark::visit(
    [&](const auto & msg) {
      using InputMsgT = std::decay_t<decltype(msg)>;
      using OutputType = typename output<InputMsgT>::type;
      m_publisher = create_publisher<OutputType>(m_output_topic, m_history_size);
      m_subscription = create_subscription<InputMsgT>(
        m_input_topic, m_history_size,
        [&](const typename InputMsgT::SharedPtr msg) {
          if (!msg) {return;}
          auto new_msg = convert(*msg);
          m_core->set_all_covariances(&new_msg);
          auto publisher = std::static_pointer_cast<rclcpp::Publisher<OutputType>>(m_publisher);
          publisher->publish(new_msg);
        });
    }, msg_variant);
}

}  // namespace covariance_insertion_nodes
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::covariance_insertion_nodes::CovarianceInsertionNode)
