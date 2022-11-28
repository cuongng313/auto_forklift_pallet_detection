cmake_minimum_required(VERSION 3.0.2)
project(multi_pallet_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  dynamic_reconfigure
  sensor_msgs
  cv_bridge
)

generate_dynamic_reconfigure_options(
  cfg/pallet_detection_reconfig.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multi_pallet_detection
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  include/eigen
)

link_directories(cfg)

add_executable(pallet_detection src/pallet_detection.cpp)
target_link_libraries(pallet_detection ${catkin_LIBRARIES} pcl_visualization)
add_dependencies(pallet_detection ${PROJECT_NAME}_gencfg)

