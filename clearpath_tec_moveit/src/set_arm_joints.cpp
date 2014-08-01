/**
Software License Agreement (proprietary)

\file      set_arm_joints.cpp
\authors   Arnold <akalmbach@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
  ros::NodeHandle nhp("~");
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();
  move_group_interface::MoveGroup group("arm");
  group.setPlannerId("PRMstarkConfigDefault");
  group.setPlanningTime(10);
  group.setStartStateToCurrentState();
  
  std::vector<double> d_joints(7, 0);
  
  nhp.param<double>("j1", d_joints[0], 0);
  nhp.param<double>("j2", d_joints[1], 0);
  nhp.param<double>("j3", d_joints[2], 0);
  nhp.param<double>("j4", d_joints[3], 0);
  nhp.param<double>("j5", d_joints[4], 0);
  nhp.param<double>("j6", d_joints[5], 0);
  nhp.param<double>("j7", d_joints[6], 0);
  
  std::vector<double> current_joint_positions = group.getCurrentJointValues();
  std::vector<double> new_joint_positions = current_joint_positions;
  for (int i = 0; i < current_joint_positions.size(); i++) {
      new_joint_positions[i] += d_joints[i];
      std::cout << "Joint " << i+1 << ": ";
      std::cout << current_joint_positions[i] << "->" << new_joint_positions[i] << std::endl;
  }
  
  group.setJointValueTarget(new_joint_positions);
  
  // plan the motion and then move the group to the sampled target 
  moveit::planning_interface::MoveGroup::Plan plan;
  bool found_plan = group.plan(plan);
    
  if (found_plan) {
    std::cout << "found a plan!" << std::endl;
    group.execute(plan);
  }
  else
    std::cout << "fail."  << std::endl;

  ros::waitForShutdown();
}
