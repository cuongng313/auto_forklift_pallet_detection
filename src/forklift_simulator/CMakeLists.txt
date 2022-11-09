cmake_minimum_required(VERSION 3.0.2)
project(forklift_simulator)

find_package(catkin REQUIRED COMPONENTS rostime roscpp rospy rosconsole roslaunch)
roslaunch_add_file_check(launch)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
   LIBRARIES forklift_simulator
   CATKIN_DEPENDS roscpp rospy std_msgs
)


add_executable(test_node src/test.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})
install(TARGETS test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(test_subs_node src/test_subs.cpp)
target_link_libraries(test_subs_node ${catkin_LIBRARIES})
install(TARGETS test_subs_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(keyboard_control_forklift src/keyboard_control_RX20_16.cpp)
target_link_libraries(keyboard_control_forklift ${catkin_LIBRARIES})
install(TARGETS keyboard_control_forklift
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}) 

add_executable(keyboard_control_container src/keyboard_control_container_door.cpp)
target_link_libraries(keyboard_control_container ${catkin_LIBRARIES})
install(TARGETS keyboard_control_container
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})   

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# install(DIRECTORY worlds
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/keyboard_control.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(FILES
  launch/forklift_sim.launch
  launch/visualize.launch
  launch/test.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
