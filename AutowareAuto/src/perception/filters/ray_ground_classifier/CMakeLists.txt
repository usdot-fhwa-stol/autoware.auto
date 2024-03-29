# Copyright 2017-2020 Apex.AI, Inc., Arm Limited
# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(ray_ground_classifier)
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)

set(use_OMP FALSE)
if(use_OMP)
  find_package(OpenMP REQUIRED)
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
else()
  set(OpenMP_FLAGS "-Wno-unknown-pragmas")
endif()

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/ray_ground_classifier/ray_aggregator.hpp
  include/ray_ground_classifier/ray_ground_classifier.hpp
  include/ray_ground_classifier/ray_ground_point_classifier.hpp
  include/ray_ground_classifier/visibility_control.hpp
  src/ray_aggregator.cpp
  src/ray_ground_point_classifier.cpp
  src/ray_ground_classifier.cpp
  src/ray_ground_classifier_types.cpp)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_ray_ground_classifier_gtest
    test/src/test_ray_ground_classifier.cpp
  )
  autoware_set_compile_options(test_ray_ground_classifier_gtest)
  target_compile_options(test_ray_ground_classifier_gtest PRIVATE -Wno-conversion -Wno-double-promotion -Wno-float-conversion)
  target_include_directories(test_ray_ground_classifier_gtest
    PRIVATE "test/include" "include"
  )
  target_link_libraries(test_ray_ground_classifier_gtest
    ${PROJECT_NAME}
    ${OpenMP_LIBS}
  )

  ament_add_gtest(test_ray_ground_classifier_raytrace_gtest
    test/src/test_ray_ground_classifier_raytrace.cpp
  )
  autoware_set_compile_options(test_ray_ground_classifier_raytrace_gtest)
  target_compile_options(test_ray_ground_classifier_raytrace_gtest PRIVATE -Wno-conversion)
  target_include_directories(test_ray_ground_classifier_raytrace_gtest
    PRIVATE "test/include" "include"
  )
  target_link_libraries(test_ray_ground_classifier_raytrace_gtest
    ${PROJECT_NAME}
    ${OpenMP_LIBS}
  )

  ament_add_gtest(test_ray_aggregator_gtest
    test/src/test_ray_aggregator.cpp
  )
  autoware_set_compile_options(test_ray_aggregator_gtest)
  target_compile_options(test_ray_aggregator_gtest PRIVATE -Wno-float-conversion -Wno-conversion)

  target_include_directories(test_ray_aggregator_gtest
    PRIVATE "test/include" "include"
  )
  target_link_libraries(test_ray_aggregator_gtest
    ${PROJECT_NAME}
    ${OpenMP_LIBS}
  )
  if(use_OMP)
    target_compile_options(test_ray_aggregator_gtest PRIVATE
    ${OpenMP_FLAGS})
  endif()
endif()

# workaround to disable sign conversion errors from sensor_msgs::PointCloud2Iterator
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wno-sign-conversion
    ${OpenMP_FLAGS}
)


# Ament Exporting
ament_auto_package()
