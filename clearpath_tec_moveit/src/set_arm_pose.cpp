/**
Software License Agreement (proprietary)

\file      set_arm_pose.cpp
\authors   Arnold <akalmbach@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#include <moveit/move_group_interface/move_group.h>
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
  group.setPoseReferenceFrame("base_link");
  group.setPlannerId("PRMstarkConfigDefault");
  group.setPlanningTime(10);
  group.setStartStateToCurrentState();
  
  double d_lx, d_ly, d_lz, d_r, d_p, d_y;
  
  nhp.param<double>("lx", d_lx, 0);
  nhp.param<double>("ly", d_ly, 0);
  nhp.param<double>("lz", d_lz, 0);
  nhp.param<double>("r", d_r, 0);
  nhp.param<double>("p", d_p, 0);
  nhp.param<double>("y", d_y, 0);
  
  geometry_msgs::PoseStamped c = group.getCurrentPose();

  std::cout << "current: " << c.pose;
  std::vector<double> rpy = group.getCurrentRPY();
  std::cout << "r: " << rpy[0];
  std::cout << ", p: " << rpy[1];
  std::cout << ", y: " << rpy[2];
  std::cout << std::endl << std::endl;
  
  geometry_msgs::PoseStamped t = c;
  t.pose.position.x += d_lx;
  t.pose.position.y += d_ly;
  t.pose.position.z += d_lz;
  double r = d_r + rpy[0];
  double p = d_p + rpy[1];
  double y = d_y + rpy[2];
  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(r, p, y), t.pose.orientation);


  std::cout << "target: " << t.pose;
  std::cout << "r: " << r;
  std::cout << ", p: " << p;
  std::cout << ", y: " << y;
  std::cout << std::endl;
  
  group.setPoseTarget(t);
  
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
