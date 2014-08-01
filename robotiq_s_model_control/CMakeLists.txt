cmake_minimum_required(VERSION 2.8.3)
project(robotiq_s_model_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
        SModel_robot_input.msg 
        SModel_robot_output.msg
)

catkin_python_setup()

generate_messages()

catkin_package(
    CATKIN_DEPENDS message_runtime
)

install(PROGRAMS nodes/SModelSimpleController.py  nodes/SModelStatusListener.py  nodes/SModelTcpNode.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
