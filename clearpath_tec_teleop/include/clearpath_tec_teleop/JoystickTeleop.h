/**
Software License Agreement (BSD)

\file      JoystickTeleop.h
\authors   Arnold <akalmbach@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the 
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/*
 * Joystick Teleop Class
 * Subscribes to Logitech Joystick output and outputs Schunk LWA velocity commands and Robotiq Gripper Open/Close Commands 
 * whenever Husky commands are not being published (i.e 'A' button is not pressed)*/

#include <ros/ros.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointValue.h>
#include <sensor_msgs/Joy.h>
#include <robotiq_s_model_control/SModel_robot_output.h>
#include <std_msgs/Bool.h>

#define NUM_JOINTS 7
#define LOWER_JOINT 2
#define MAX_VEL 0.2
#define NUM_GRIPPER_MODES 4

enum {BASIC_MODE=0, PINCH_MODE=1, WIDE_MODE=2, SCISSOR_MODE=3};

class JoystickTeleop
{
    public:
        JoystickTeleop (ros::NodeHandle &n);
        void sendArmCommand(const ros::TimerEvent&);
        
   private:
        void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg) ;
        
        ros::NodeHandle node_;
        brics_actuator::JointValue joint_value[NUM_JOINTS];
        bool gripperActivated;
        int gripper_mode;
        bool arm_enabled;
        robotiq_s_model_control::SModel_robot_output gripper_command;
        brics_actuator::JointVelocities sample_command;

        ros::Publisher joint_vel_pub;
        ros::Publisher gripper_pub;
        ros::Publisher arm_light_pub;
        ros::Publisher mast_light_pub;
        ros::Subscriber joy_sub; 

        std_msgs::Bool arm_light;
        std_msgs::Bool mast_light;

        bool left_trigger_nonzero;
        bool right_trigger_nonzero;
};
