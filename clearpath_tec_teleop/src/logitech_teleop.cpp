/**
Software License Agreement (BSD)

\file      logitech_teleop.cpp
\authors    Arnold <akalmbach@clearpathrobotics.com>
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
#include "clearpath_tec_teleop/JoystickTeleop.h"

JoystickTeleop::JoystickTeleop (ros::NodeHandle &n): node_(n)
{
    left_trigger_nonzero = false;
    right_trigger_nonzero = false;

    for (int i=0;i<NUM_JOINTS;i++) {
        joint_value[i].value = 0;
        joint_value[i].unit = "rad/s";
        joint_value[i].joint_uri = "N/A";
        joint_value[i].timeStamp = ros::Time::now();
    }

    gripperActivated = false;
    gripper_mode = BASIC_MODE;
    arm_enabled = false;
    sample_command.velocities.resize(NUM_JOINTS);

    joint_vel_pub = node_.advertise<brics_actuator::JointVelocities>("arm_controller/command_vel",1000);
    arm_light_pub = node_.advertise<std_msgs::Bool>("/husky_01/arm_light",1);
    mast_light_pub = node_.advertise<std_msgs::Bool>("/husky_01/mast_light",1);
    joy_sub = node_.subscribe<sensor_msgs::Joy>("joy", 1000, &JoystickTeleop::joystickCallback,this);
    gripper_pub = node_.advertise<robotiq_s_model_control::SModel_robot_output>("SModelRobotOutput",1000);
    arm_light.data = false; 
    mast_light.data = false;

}

void JoystickTeleop::sendArmCommand(const ros::TimerEvent&) 
{
    for (int i=0;i<NUM_JOINTS;i++) {
        joint_value[i].timeStamp = ros::Time::now();
        sample_command.velocities[i] = joint_value[i];
    }

    gripper_pub.publish (gripper_command);
    if (arm_enabled)
        joint_vel_pub.publish(sample_command);
}


void JoystickTeleop::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg) 
{
   if (msg->buttons[0] != 1 && // husky is not being actively controlled
       msg->buttons[3] == 1) { // and the arm is enabled
        arm_enabled = true;
        double left_trigger_value = msg->axes[2];
        double right_trigger_value = msg->axes[5];
        double wrist_l_val = 0;
        double wrist_r_val = 0;

        //triggers swing from -1 to 1, rescale to 0-1
        wrist_r_val = -(right_trigger_value - 1) /2.0;
        wrist_l_val = -(left_trigger_value - 1)/2.0;

        //slight joystick driver oddity handled here, trigger always reads zero unless its pressed after which it swings from -1 to 1
        if (left_trigger_value==0 && !left_trigger_nonzero)  //if left trigger value is zero, and it has never become non zero before then read zero
            wrist_l_val = 0;
        else if(left_trigger_value !=0 && left_trigger_nonzero) //well you are nonzero now
            left_trigger_nonzero = true;

        if (right_trigger_value==0 && !right_trigger_nonzero)  //same thing with right trigger
            wrist_r_val = 0;
        else if(right_trigger_value !=0 && right_trigger_nonzero) //well you are nonzero now
            right_trigger_nonzero = true;

        joint_value[0].value = -msg->axes[3] * MAX_VEL;
        joint_value[1].value = -msg->axes[4] * MAX_VEL;
        joint_value[2].value = -msg->axes[0] * MAX_VEL;
        joint_value[3].value = -msg->axes[1] * MAX_VEL;
        joint_value[4].value = msg->axes[6] * MAX_VEL;
        joint_value[5].value = msg->axes[7] * MAX_VEL;
        joint_value[6].value = (wrist_r_val - wrist_l_val) * MAX_VEL;
    }
    else arm_enabled = false;

    if (msg->buttons[1] == 1) { // switch gripping mode
        if (!gripperActivated) {
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSPA = 255;
            gripper_command.rFRA = 150;
            gripperActivated = true;
        }
        else{
            gripper_mode = (gripper_mode+1)%NUM_GRIPPER_MODES;
            gripper_command.rMOD = gripper_mode;
            if (gripper_mode == BASIC_MODE)
                ROS_INFO("Gripper mode set to BASIC");
            if (gripper_mode == PINCH_MODE)
                ROS_INFO("Gripper mode set to PINCH");
            if (gripper_mode == WIDE_MODE)
                ROS_INFO("Gripper mode set to WIDE");
            if (gripper_mode == SCISSOR_MODE)
                ROS_INFO("Gripper mode set to SCISSOR");
        }
    }

    if (msg->buttons[4] == 1) { //close gripper
        if (!gripperActivated) {
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSPA = 255;
            gripper_command.rFRA = 150;
            gripperActivated = true;
        }
        else{
            gripper_command.rPRA = 255;
        }
    }
    
    if (msg->buttons[5] == 1) { //open gripper
        if (!gripperActivated) {
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSPA = 255;
            gripper_command.rFRA = 150;
            gripperActivated = true;
        }
        else {
            gripper_command.rPRA = 0;
        }
    }


    if (msg->buttons[4] ==1) {
        arm_light.data = !(arm_light.data);
        arm_light_pub.publish(arm_light);
    }

    if (msg->buttons[5] == 1) {
        mast_light.data = !(mast_light.data);
        mast_light_pub.publish(mast_light);
    }

}


int main(int argc, char **argv)
{
    ros::init (argc, argv, "logitech_teleop");
    ros::NodeHandle nh;

    JoystickTeleop teleop_obj(nh);

    //send arm commands at 1/0.02 = 50Hz  
    ros::Timer timer = nh.createTimer(ros::Duration(0.02), &JoystickTeleop::sendArmCommand, &teleop_obj);
    ros::spin();
    return 0;
}
