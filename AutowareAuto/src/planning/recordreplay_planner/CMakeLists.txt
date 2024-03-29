
#
# Modifications copyright (C) 2021 Leidos
#

cmake_minimum_required(VERSION 3.6)

project(recordreplay_planner)
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Build library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/recordreplay_planner/recordreplay_planner.cpp
)
autoware_set_compile_options_reduced_warning(${PROJECT_NAME})

### Test
if(BUILD_TESTING)
  # Linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ## Unit test
  ament_add_gtest(recordreplay_planner_unit_tests
    test/gtest_main.cpp
    test/sanity_checks.cpp)
    autoware_set_compile_options_reduced_warning(recordreplay_planner_unit_tests)
  target_link_libraries(recordreplay_planner_unit_tests ${PROJECT_NAME})
  target_include_directories(recordreplay_planner_unit_tests PRIVATE "include")
endif()

ament_auto_package()
