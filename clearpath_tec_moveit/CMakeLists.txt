cmake_minimum_required(VERSION 2.8.3)
project(clearpath_tec_moveit)

find_package(catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(move_joints src/set_arm_joints.cpp)
target_link_libraries(move_joints ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_joints DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_pose src/set_arm_pose.cpp)
target_link_libraries(move_pose ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_pose DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
