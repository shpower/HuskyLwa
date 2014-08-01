/**
Software License Agreement (proprietary)

\file      lwa_plugin.cpp
\authors   Arnold <akalmbach@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "brics_actuator/JointVelocities.h"
#include "cob_srvs/Trigger.h"
#include "cob_srvs/SetOperationMode.h"

#include <lwa_plugin/lwa_plugin.h>

#include <boost/lexical_cast.hpp>
#include <ros/time.h>

using namespace gazebo;

LWAPlugin::LWAPlugin()
{
}

LWAPlugin::~LWAPlugin()
{
}

void LWAPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    this->model_ = _parent;
    this->world_ = this->model_->GetWorld();

    this->node_namespace_ = "";
    if (_sdf->HasElement("robotNamespace"))
    this->node_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

    // There is no Element::Get<std::vector<T> > so have to parse it ourselves
    std::vector<double> initial_joint_positions(7, 0);
    std::string initial_joint_positions_s;
    if (_sdf->HasElement("initialJointPositions"))
        initial_joint_positions_s = _sdf->GetElement("initialJointPositions")->Get<std::string>();
    std::stringstream tmp(initial_joint_positions_s);
    std::string s;
    int j = 0;
    while (getline(tmp, s, ' '))
    {
        if (j < initial_joint_positions.size())
            initial_joint_positions[j] = (boost::lexical_cast<double>(s));
        else
            ROS_ERROR("Extra values were specified for initial joint positions");
        j++;
    }
         
    for (int i = 0; i < 7; i++)
    {
        std::string joint_name = "arm_" + boost::lexical_cast<std::string>(i+1) + "_joint";
        std::string q = "joint_" + boost::lexical_cast<std::string>(i+1);
        
        if (_sdf->HasElement(q))
            joint_name = _sdf->GetElement("joint_" + boost::lexical_cast<std::string>(i+1))->Get<std::string>();
        joint_names_.push_back(joint_name);
        
        js_.name.push_back(joint_name);
        js_.position.push_back(initial_joint_positions[i]);
        js_.velocity.push_back(0);
        js_.effort.push_back(0);
        joint_targets_.push_back(0);
        joint_velocities_.push_back(0);
        
        joints_[i] = model_->GetJoint(joint_name);
        
        if (joints_[i]) 
        {
            set_joints_[i] = true;
            joint_targets_[i] = initial_joint_positions[i];
            joints_[i]->SetAngle(0, gazebo::math::Angle(initial_joint_positions[i]));
        }
        else set_joints_[i] = false;        
    }
    
    if (set_joints_[6]) joints_[6]->SetProvideFeedback(true);

    base_geom_name_ = "base_link";
    if (_sdf->HasElement("baseGeom"))
        base_geom_name_ = _sdf->GetElement("baseGeom")->Get<std::string>();
    base_geom_ = model_->GetChildCollision(base_geom_name_);

    // Get the name of the parent model
    std::string modelName = _sdf->GetParent()->Get<std::string>("name");

    if (!ros::isInitialized())
    {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "gazebo_lwa", ros::init_options::AnonymousName);
    }

    rosnode_ = new ros::NodeHandle( node_namespace_ );

    cmd_vel_sub_ = rosnode_->subscribe("command_vel", 1, &LWAPlugin::OnCmdVel, this );
    joint_state_pub_ = rosnode_->advertise<sensor_msgs::JointState>("/joint_states", 1);
    opmode_pub_ = rosnode_->advertise<std_msgs::String>("current_operationmode", 1000);
    trajectory_state_pub_ = rosnode_->advertise<control_msgs::JointTrajectoryControllerState>("state", 1000);
    force_torque_pub_ = rosnode_->advertise<geometry_msgs::WrenchStamped>("/ftm/force_values", 1);

       
    // for cob_trajectory_controller
    set_operation_mode_srv_ = rosnode_->advertiseService("set_operation_mode", &LWAPlugin::SetOperationModeSrvCallback, this);
    init_srv_ = rosnode_->advertiseService("init", &LWAPlugin::TriggerSrvCallback, this);

    prev_update_time_ = 0;
    last_cmd_time_ = 0;

    //initialize time and odometry position
    prev_update_time_ = last_cmd_time_ = this->world_->GetSimTime();
    
    // Listen to the update event. This event is broadcast every simulation iteration.
    this->spinner_thread_ = new boost::thread( boost::bind( &LWAPlugin::spin, this) );
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&LWAPlugin::UpdateChild, this));
}


void LWAPlugin::UpdateChild()
{
    common::Time time_now = this->world_->GetSimTime();
    common::Time step_time = time_now - prev_update_time_;
    prev_update_time_ = time_now;

    for (int i = 0; i < 7; i++)
    {
        if (set_joints_[i])
        {
            
            js_.position[i] = joints_[i]->GetAngle(0).Radian();
            js_.velocity[i] = joints_[i]->GetVelocity(0);
            
            joints_[i]->SetAngle(0, joint_targets_[i]);
            // TODO(akalmbach): parameterize this
            joints_[i]->SetMaxForce(0, 100);

        }
    }

    // simulate force-torque sensor
    gazebo::physics::JointWrench force_torque = joints_[6]->GetForceTorque(0u);
    geometry_msgs::WrenchStamped force_torque_msg;
    force_torque_msg.header.stamp.sec = time_now.sec;
    force_torque_msg.header.stamp.nsec = time_now.nsec;
    force_torque_msg.header.frame_id = "arm_gripper_link";
    
    force_torque_msg.wrench.force.x = force_torque.body2Force.x;
    force_torque_msg.wrench.force.y = force_torque.body2Force.y;
    force_torque_msg.wrench.force.z = force_torque.body2Force.z;
    force_torque_msg.wrench.torque.x = force_torque.body2Torque.x;
    force_torque_msg.wrench.torque.y = force_torque.body2Torque.y;
    force_torque_msg.wrench.torque.z = force_torque.body2Torque.z;
    force_torque_pub_.publish(force_torque_msg);
    
    
    // publish joint states
    js_.header.stamp.sec = time_now.sec;
    js_.header.stamp.nsec = time_now.nsec;
    joint_state_pub_.publish( js_ );
    
    opmode_.data = "velocity";
    opmode_pub_.publish(opmode_);
    
    trajectory_state_.header.stamp.sec = time_now.sec;
    trajectory_state_.header.stamp.nsec = time_now.nsec;
    trajectory_state_.joint_names = joint_names_;
    trajectory_state_.desired.positions = joint_targets_;
    trajectory_state_.desired.velocities = joint_velocities_;
    trajectory_state_.actual.positions = js_.position;
    trajectory_state_.actual.velocities = js_.velocity;
    trajectory_state_pub_.publish(trajectory_state_);
    
}


void LWAPlugin::OnCmdVel( const brics_actuator::JointVelocitiesConstPtr &msg)
{
    common::Time time_now = this->world_->GetSimTime();
    common::Time step_time = time_now - last_cmd_time_;
    last_cmd_time_ = time_now;

    assert (msg->velocities.size() == 7);

    for (int i = 0; i < 7; i++)
    {
        //std::cout << joint_targets_[i] <<  " += " << msg->velocities[i].value << " * (" << step_time.nsec << " / 100000000.0)" << std::endl;
        joint_velocities_[i] = msg->velocities[i].value;
        joint_targets_[i] += msg->velocities[i].value * (step_time.nsec/1000000000.0);
        
        double upper_limit = 0;
        double lower_limit = 0;
        if (set_joints_[i])
        {
            upper_limit = joints_[i]->GetUpperLimit(0).Radian();
            lower_limit = joints_[i]->GetLowerLimit(0).Radian();
        }
        if (joint_targets_[i] >= upper_limit) joint_targets_[i] = upper_limit - 0.001;
        if (joint_targets_[i] <= lower_limit) joint_targets_[i] = lower_limit + 0.001;
    }
}


bool LWAPlugin::TriggerSrvCallback(cob_srvs::Trigger::Request &req,
		 cob_srvs::Trigger::Response &res) {
  // this should indicate whether the model was successfully initialized
  // for now it does nothing
  res.success.data = true;
  res.error_message.data = "";
  return true;
}

bool LWAPlugin::SetOperationModeSrvCallback(cob_srvs::SetOperationMode::Request &req,
			      cob_srvs::SetOperationMode::Response &res) {
  res.success.data = true;  // for now this service is just a dummy, not used elsewhere
  // res.error_message.data = "";
  return true;
}

void LWAPlugin::spin()
{
  while(ros::ok()) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(LWAPlugin);
