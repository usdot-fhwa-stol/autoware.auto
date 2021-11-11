cmake_minimum_required(VERSION 3.5)
project(lexus_rx_450h_description)
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
  meshes
  urdf
)
