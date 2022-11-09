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
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES multi_pallet_detection
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  include/eigen
)

# add_executable(plane_segmentation src/main.cpp src/plane_segmentation.cpp)
# target_link_libraries(plane_segmentation ${catkin_LIBRARIES} pcl_visualization)

add_executable(pallet_detection src/pallet_detection.cpp)
target_link_libraries(pallet_detection ${catkin_LIBRARIES})

