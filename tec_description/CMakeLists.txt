cmake_minimum_required(VERSION 2.8.3)
project(tec_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS gazebo_plugins gazebo_ros gazebo_msgs roscpp std_msgs brics_actuator cob_srvs control_msgs)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_plugins gazebo_ros gazebo_msgs roscpp std_msgs brics_actuator cob_srvs control_msgs
  DEPENDS system_lib 
)

include_directories(include)
include_directories(include/tec_plugin)
include_directories(include/lwa_plugin)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Declare a cpp library
add_library(tec_husky_gazebo_plugin
  src/tec_plugin.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(tec_husky_gazebo_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# Declare a cpp library
add_library(lwa_gazebo_plugin
  src/lwa_plugin.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(lwa_gazebo_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

install(
  DIRECTORY launch worlds meshes calibration
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS tec_husky_gazebo_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS lwa_gazebo_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
