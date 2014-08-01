cmake_minimum_required(VERSION 2.8.3)

project(clearpath_tec_teleop)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  brics_actuator
  cob_srvs
  robotiq_s_model_control
  geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs brics_actuator cob_srvs robotiq_s_model_control
)

include_directories(include)
include_directories(include/clearpath_tec_teleop)


link_libraries(${catkin_LIBRARIES})

add_executable(logitech_teleop src/logitech_teleop.cpp)
#add_executable(falcon_spacenav_teleop src/falcon_spacenav_teleop.cpp)

install(
  DIRECTORY launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
