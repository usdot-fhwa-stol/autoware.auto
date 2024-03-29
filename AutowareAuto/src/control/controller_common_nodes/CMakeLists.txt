cmake_minimum_required(VERSION 3.6)

project(controller_common_nodes)
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

### Build
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/controller_common_nodes/controller_base_node.cpp)
autoware_set_compile_options(${PROJECT_NAME})
# Setting a system include is needed to avoid having an explicit Eigen dependency.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()
  # Unit test
  ament_add_gtest(controller_common_nodes_unit_tests
    test/bad_cases.cpp
    test/error_handling.cpp
    test/gtest_main.cpp
    test/sanity_check.cpp
    test/transform.cpp)
  autoware_set_compile_options(controller_common_nodes_unit_tests)
  target_compile_options(controller_common_nodes_unit_tests PRIVATE -Wno-double-promotion)
  target_link_libraries(controller_common_nodes_unit_tests ${PROJECT_NAME})
  add_dependencies(controller_common_nodes_unit_tests ${PROJECT_NAME})
endif()

ament_auto_package()
