#!/bin/bash

#  Copyright (C) 2021 LEIDOS.
#
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

cd ~/
# Source Environment variables
# Source ros2
if [[ ! -z "$ROS2_PACKAGES" ]]; then
    echo "Sourcing previous build for incremental build start point..."
    source /opt/carma/install/setup.bash
else
    echo "Sourcing base image for full build..."
    source /opt/ros/humble/setup.bash
fi

# Build
# NOTE: The following packages are excluded from the build process because
# they are not needed for the CARMA Platform and dependent on lanelet2
# which is not included in autoware auto to avoid circular dependency on
# autoware.ai which has our lanelet2 version
if [[ ! -z "$ROS2_PACKAGES" ]]; then
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-above $ROS2_PACKAGES --packages-ignore \
    had_map_utils autoware_auto_launch autoware_demos lanelet2_map_provider off_map_obstacles_filter \
    off_map_obstacles_filter_nodes behavior_planner lane_planner lanelet2_global_planner \
    lanelet2_global_planner_nodes parking_planner parking_planner_nodes
else
    # Install dependencies
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-ignore \
    had_map_utils autoware_auto_launch autoware_demos lanelet2_map_provider off_map_obstacles_filter \
    off_map_obstacles_filter_nodes behavior_planner lane_planner lanelet2_global_planner \
    lanelet2_global_planner_nodes parking_planner parking_planner_nodes
fi
