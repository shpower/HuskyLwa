/**
Software License Agreement (proprietary)

\file      lwa_plugin.h
\authors   Arnold <akalmbach@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#ifndef GAZEBO_ROS_CREATE_H
#define GAZEBO_ROS_CREATE_H

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include "std_msgs/String.h"
#include <geometry_msgs/WrenchStamped.h>
#include "control_msgs/JointTrajectoryControllerState.h"
#include "cob_srvs/Trigger.h"
#include "cob_srvs/SetOperationMode.h"
#include "brics_actuator/JointVelocities.h"

#include <ros/ros.h>


namespace gazebo
{
  class LWAPlugin : public ModelPlugin
  {
    public: 
      LWAPlugin();
      virtual ~LWAPlugin();
          
      virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf );

      virtual void UpdateChild();
  
    private:

      void OnContact(const std::string &name, const physics::Contact &contact);
      void OnCmdVel( const brics_actuator::JointVelocitiesConstPtr &msg);
      bool TriggerSrvCallback(cob_srvs::Trigger::Request &req,
                              cob_srvs::Trigger::Response &res);

      bool SetOperationModeSrvCallback(cob_srvs::SetOperationMode::Request &req,
                                       cob_srvs::SetOperationMode::Response &res);


      /// Parameters
      std::string node_namespace_;
      std::string base_geom_name_;

      ros::NodeHandle *rosnode_;
  
      ros::Publisher joint_state_pub_;
      ros::Publisher opmode_pub_;
      ros::Publisher trajectory_state_pub_;
      ros::Publisher force_torque_pub_;      
      ros::Subscriber cmd_vel_sub_;
      ros::ServiceServer set_operation_mode_srv_;
      ros::ServiceServer init_srv_;
      
      sensor_msgs::JointState js_;
      std_msgs::String opmode_;
      control_msgs::JointTrajectoryControllerState trajectory_state_;

      physics::WorldPtr world_;
      physics::ModelPtr model_;
     
      // time of last gazebo update
      common::Time prev_update_time_;
      // gazebo time of last ros message
      common::Time last_cmd_time_;

      bool set_joints_[7];
      physics::JointPtr joints_[7];
      std::vector<double> joint_targets_;
      std::vector<double> joint_velocities_;
      std::vector<std::string> joint_names_;
      physics::CollisionPtr base_geom_;


      void spin();
      boost::thread *spinner_thread_;

      event::ConnectionPtr contact_event_;

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;
  };
}
#endif
