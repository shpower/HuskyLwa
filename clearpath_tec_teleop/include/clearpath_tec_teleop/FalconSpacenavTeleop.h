/**
Software License Agreement (BSD)

\file      FalconSpacenavTeleop.h
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
#include <ros/ros.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointValue.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Wrench.h>
#include "rosfalcon/falconForces.h"
#include <geometry_msgs/Twist.h>
#include <robotiq_s_model_control/SModel_robot_output.h>
#include <cob_srvs/Trigger.h>

#define NUM_JOINTS 7
#define MAX_VEL 0.2
#define MAX_SPACENAV_TWIST 350
#define MAX_FALCON_INPUT 0.03
#define SPACENAV_THRESHOLD 100
#define FALCON_THRESHOLD 0.005
#define MAX_HUSKY_VEL 0.5 
#define FALCON_CALIB_THRESH 1000
#define FORCE_SCALE 0.05

class FalconSpacenavTeleop
{
    public:
        FalconSpacenavTeleop (ros::NodeHandle &n): node_(n) {

            gripperActivated = false;
            calibrate_falcon = true;
            ROS_INFO("Calibrating Novint Falcon Device, Please do not touch!");
            falcon_readings = 0;
            sample_command.velocities.resize(NUM_JOINTS);
            falcon_x_bias = 0;
            falcon_y_bias = 0;
            falcon_z_bias = 0;

            joint_vel_pub = node_.advertise<brics_actuator::JointVelocities>("arm_controller/cmd_vel",1000);
            gripper_pub = node_.advertise<robotiq_s_model_control::SModel_robot_output>("SModelRobotOutput",1000);
//            husky_vel_pub = node_.advertise<geometry_msgs::Twist>("/husky/cmd_vel",1000); //cannot operate husky at the same time
            falcon_force_pub = node_.advertise<rosfalcon::falconForces>("falcon/force_values",1000);
            spacenav_sub = node_.subscribe("spacenav/twist",1000, &FalconSpacenavTeleop::spacenavCallback,this);
            spacenav_bt_sub = node_.subscribe("spacenav/joy",1000,&FalconSpacenavTeleop::spacenavBtCallback,this);
            falcon_sub = node_.subscribe("falconJoy",1000,&FalconSpacenavTeleop::falconCallback,this);
            ftm_sub = node_.subscribe("ftm/force_values_base",1000,&FalconSpacenavTeleop::ftmCallback,this);
            //husky_vel_cmd.angular.z = 0;
            //husky_vel_cmd.linear.x = 0;
            ROS_INFO("Calibrating FTM Module");
            ros::ServiceClient client = node_.serviceClient<cob_srvs::Trigger>("ftm/calibrate");
            cob_srvs::Trigger::Request req;
            cob_srvs::Trigger::Response res;
            client.call(req,res);
            ROS_INFO("FTM Calibrated");

 
        }

        void ftmCallback(const geometry_msgs::Wrench &msg)
        {
              falcon_force_cmd.X.data = msg.force.x * FORCE_SCALE;
              falcon_force_cmd.Y.data = msg.force.y * FORCE_SCALE;
              falcon_force_cmd.Z.data = msg.force.z * FORCE_SCALE;
              falcon_force_pub.publish(falcon_force_cmd);
        }


        void sendArmCommand(const ros::TimerEvent&) 
        {
            for (int i=0;i<NUM_JOINTS;i++) {
                joint_value[i].timeStamp = ros::Time::now();
                sample_command.velocities[i] = joint_value[i];
            }

            gripper_pub.publish (gripper_command);
            joint_vel_pub.publish(sample_command);
            //husky_vel_pub.publish(husky_vel_cmd);

        }
   private:
        ros::NodeHandle node_;
        bool gripperActivated;
        bool calibrate_falcon;
        bool calibrate_ftm_bias;
        int falcon_readings;

        //falcon position biases (due to imperfect position controller) 
        double falcon_x_bias;
        double falcon_y_bias;
        double falcon_z_bias;

        //ftm force biases
        double force_x_bias;
        double force_y_bias;
        double force_z_bias; 

        robotiq_s_model_control::SModel_robot_output gripper_command;

        brics_actuator::JointValue joint_value[NUM_JOINTS];
        brics_actuator::JointVelocities sample_command;
        //geometry_msgs::Twist husky_vel_cmd;
        rosfalcon::falconForces falcon_force_cmd;

        ros::Publisher joint_vel_pub;
        ros::Publisher gripper_pub;
        //ros::Publisher husky_vel_pub;
        ros::Publisher falcon_force_pub;
        ros::Subscriber spacenav_sub;
        ros::Subscriber spacenav_bt_sub;
        ros::Subscriber falcon_sub;
        ros::Subscriber ftm_sub;

        void spacenavBtCallback(const sensor_msgs::Joy& msg)
        {
            if (msg.buttons[0] == 1) { //close gripper
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
            
            if (msg.buttons[1] == 1) { //open gripper
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
        }


        void spacenavCallback(const geometry_msgs::Twist& msg) 
        {
            //husky_vel_cmd.angular.z = deadzone(msg.angular.z,SPACENAV_THRESHOLD,MAX_HUSKY_VEL,MAX_SPACENAV_TWIST);
            //husky_vel_cmd.linear.x = -deadzone(msg.linear.z,SPACENAV_THRESHOLD,MAX_HUSKY_VEL,MAX_SPACENAV_TWIST); //this direction felt more natural
            joint_value[1].value = deadzone(msg.linear.z,SPACENAV_THRESHOLD,MAX_VEL,MAX_SPACENAV_TWIST); 
            joint_value[2].value = deadzone(msg.angular.z,SPACENAV_THRESHOLD,MAX_VEL,MAX_SPACENAV_TWIST); 
            joint_value[3].value = deadzone(msg.linear.x,SPACENAV_THRESHOLD,MAX_VEL,MAX_SPACENAV_TWIST);
            joint_value[4].value = deadzone(msg.linear.y,SPACENAV_THRESHOLD,MAX_VEL,MAX_SPACENAV_TWIST);
            joint_value[5].value = deadzone(msg.angular.y,SPACENAV_THRESHOLD,MAX_VEL,MAX_SPACENAV_TWIST);
            joint_value[6].value = deadzone(msg.angular.x,SPACENAV_THRESHOLD,MAX_VEL,MAX_SPACENAV_TWIST);
        }

        double deadzone(double input, double min_threshold, double max_val, double max_input)
        {
            double output = 0;
            if (fabs(input) > min_threshold) {
                output = ((input>0) ? 1 : ((input<0) ? -1 : 0)) * (fabs(input) - min_threshold); //reset so min threshold is 0
                output = output * (max_val/(max_input - min_threshold)); //remap to scale of the output
            }
            return output;
        }



        void falconCallback(const sensor_msgs::Joy& msg)
        {
            if (calibrate_falcon) {
                falcon_x_bias = (falcon_x_bias + msg.axes[0])/2;
                falcon_y_bias = (falcon_y_bias + msg.axes[1])/2;
                falcon_z_bias = (falcon_z_bias + msg.axes[2])/2;
                if (++falcon_readings > FALCON_CALIB_THRESH) {
                    ROS_INFO("Calibration of Novint Falcon Device completed, you may now operate it!");
                    calibrate_falcon = false;
                }
            }
            else {
                double x_read = msg.axes[0] - falcon_x_bias;
                double y_read = msg.axes[1] - falcon_y_bias;
                double z_read = msg.axes[2] - falcon_z_bias;

                //joint_value[0].value = deadzone(z_read,FALCON_THRESHOLD,MAX_VEL,MAX_FALCON_INPUT);


            }
        }
};
