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

#include "lane_planner_nodes/lane_planner_node.hpp"
#include <had_map_utils/had_map_conversion.hpp>

#include <string>
#include <memory>

namespace autoware
{
namespace lane_planner_nodes
{

using motion::planning::trajectory_smoother::TrajectorySmootherConfig;

LanePlannerNode::LanePlannerNode(const rclcpp::NodeOptions & node_options)
: TrajectoryPlannerNodeBase{"lane_planner", "plan_lane_trajectory", node_options}
{
  const VehicleConfig vehicle_param{
    static_cast<Real>(declare_parameter("vehicle.cg_to_front_m", rclcpp::PARAMETER_DOUBLE).get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.cg_to_rear_m", rclcpp::PARAMETER_DOUBLE).get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.front_corner_stiffness", rclcpp::PARAMETER_DOUBLE).get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.rear_corner_stiffness", rclcpp::PARAMETER_DOUBLE).get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.mass_kg", rclcpp::PARAMETER_DOUBLE).get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.yaw_inertia_kgm2", rclcpp::PARAMETER_DOUBLE).get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.width_m", rclcpp::PARAMETER_DOUBLE).get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.front_overhang_m", rclcpp::PARAMETER_DOUBLE).get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.rear_overhang_m", rclcpp::PARAMETER_DOUBLE).get<float32_t>())
  };
  const TrajectorySmootherConfig config{
    static_cast<float32_t>(
      declare_parameter("gaussian_smoother.standard_deviation", rclcpp::PARAMETER_DOUBLE).get<float64_t>()),
    static_cast<uint32_t>(declare_parameter("gaussian_smoother.kernel_size", rclcpp::PARAMETER_INTEGER).get<uint64_t>())
  };
  const lane_planner::LanePlannerConfig planner_config{
    static_cast<float32_t>(
      declare_parameter("lane_planner.trajectory_resolution", rclcpp::PARAMETER_DOUBLE).get<float64_t>())
  };

  m_planner = std::make_unique<lane_planner::LanePlanner>(vehicle_param, config, planner_config);
}

HADMapService::Request LanePlannerNode::create_map_request(const HADMapRoute & had_map_route)
{
  (void) had_map_route;

  // TODO(mitsudome-r): replace it with bounded request
  HADMapService::Request request;
  request.requested_primitives.push_back(HADMapService::Request::FULL_MAP);
  // const auto goal = goal_handle->get_goal()->sub_route.goal_point;
  // const auto start = goal_handle->get_goal()->sub_route.start_point;
  // request->geom_lower_bound[0] = std::min(start.x, goal.x);
  // request->geom_lower_bound[1] = std::min(start.y, goal.y);
  // request->geom_upper_bound[0] = std::max(start.x, goal.x);
  // request->geom_upper_bound[1] = std::max(start.y, goal.y);

  return request;
}

Trajectory LanePlannerNode::plan_trajectory(
  const HADMapRoute & had_map_route,
  const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  return m_planner->plan_trajectory(had_map_route, lanelet_map_ptr);
}


}  // namespace lane_planner_nodes
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::lane_planner_nodes::LanePlannerNode)
