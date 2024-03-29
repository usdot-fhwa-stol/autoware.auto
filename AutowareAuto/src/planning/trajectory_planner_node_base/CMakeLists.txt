# Copyright 2020 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Modifications copyright (C) 2021 Leidos
#

cmake_minimum_required(VERSION 3.5)

project(trajectory_planner_node_base)
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(TRAJECTORY_PLANNER_NODE_BASE_LIB_SRC
  src/trajectory_planner_node_base.cpp
)

set(TRAJECTORY_PLANNER_NODE_BASE_LIB_HEADERS
  include/trajectory_planner_node_base/trajectory_planner_node_base.hpp
  include/trajectory_planner_node_base/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${TRAJECTORY_PLANNER_NODE_BASE_LIB_SRC}
  ${TRAJECTORY_PLANNER_NODE_BASE_LIB_HEADERS}
)
autoware_set_compile_options_reduced_warning(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_trajectory_planner_node_base.cpp)
  set(TEST_TRAJECTORY_PLANNER_NODE_BASE_EXE test_trajectory_planner_node_base)
  ament_add_gtest(${TEST_TRAJECTORY_PLANNER_NODE_BASE_EXE} ${TEST_SOURCES})
  autoware_set_compile_options_reduced_warning(${TEST_TRAJECTORY_PLANNER_NODE_BASE_EXE})
  target_link_libraries(${TEST_TRAJECTORY_PLANNER_NODE_BASE_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package()
