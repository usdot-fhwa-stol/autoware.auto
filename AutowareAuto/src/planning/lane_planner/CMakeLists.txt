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

project(lane_planner)
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(LANE_PLANNER_LIB_SRC
  src/lane_planner.cpp
)

set(LANE_PLANNER_LIB_HEADERS
  include/lane_planner/lane_planner.hpp
  include/lane_planner/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${LANE_PLANNER_LIB_SRC}
  ${LANE_PLANNER_LIB_HEADERS}
)
autoware_set_compile_options_reduced_warning(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_lane_planner.cpp)
  set(TEST_LANE_PLANNER_EXE test_lane_planner)
  ament_add_gtest(${TEST_LANE_PLANNER_EXE} ${TEST_SOURCES})
  autoware_set_compile_options_reduced_warning(${TEST_LANE_PLANNER_EXE})
  target_link_libraries(${TEST_LANE_PLANNER_EXE} ${PROJECT_NAME})
  add_dependencies(${TEST_LANE_PLANNER_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package()
