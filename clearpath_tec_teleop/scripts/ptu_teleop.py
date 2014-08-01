#!/usr/bin/python
# Software License Agreement (BSD) 
#
# @author     Arnold <akalmbach@clearpathrobotics.com>
# @copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or
#   promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from math import pi
import rospy

from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from numpy import radians as rad

class PTUTeleop:
    def __init__(self):
        rospy.init_node('ptu_teleop')

        pan_lim = rospy.get_param('~pan_lim', 3.14)
        tilt_hi_lim = rospy.get_param('~tilt_hi_lim', pi)
        tilt_lo_lim = -rospy.get_param('~tilt_lo_lim', -pi)
        
        tilt_scale = rospy.get_param('~tilt_scale', 1)
        pan_scale = rospy.get_param('~pan_scale', 1)
        
        self.pan_speed = rospy.get_param('~pan_speed', 1)
        self.tilt_speed = rospy.get_param('~tilt_speed', 1)

        self.ptu_button = rospy.get_param('~ptu_button', 2)
        self.home_button = rospy.get_param('~home_button', 7)

        # Desired PTU behaviour - set by joystick callback
        self.des_pan_vel = 0
        self.des_tilt_vel = 0
        self.ptu_changed = False
        self.ptu_reset = False
        pan_setpt = 0
        tilt_setpt = 0

        cmd_ptu_pub = rospy.Publisher('ptu/cmd', JointState)

        announce_pub = rospy.Publisher('/clearpath/announce/teleops',
                                       String, latch=True)
        announce_pub.publish(rospy.get_namespace());

        rospy.Subscriber("joy", Joy, self.callback)
        freq = rospy.get_param('~hz',50)
        rate = rospy.Rate(freq)

        while not rospy.is_shutdown():
            rate.sleep()

            if self.ptu_reset or self.des_pan_vel != 0 or self.des_tilt_vel != 0:
                ptu = JointState()
                ptu.name.append("Pan")
                ptu.name.append("Tilt")
                # If we want to home, home
                if self.ptu_reset:
                    pan_setpt = 0
                    tilt_setpt = 0
                else:
                    pan_setpt += pan_scale*self.des_pan_vel*freq/1000.0
                    tilt_setpt += tilt_scale*self.des_tilt_vel*freq/1000.0
                    
                    pan_setpt = min(pan_lim,pan_setpt)
                    pan_setpt = max(-pan_lim,pan_setpt)
                    tilt_setpt = min(tilt_hi_lim,tilt_setpt)
                    tilt_setpt = max(-tilt_lo_lim,tilt_setpt)
                    
                ptu.position.append(pan_setpt)
                ptu.position.append(tilt_setpt)
                ptu.velocity.append(rad(self.pan_speed))
                ptu.velocity.append(rad(self.tilt_speed))
                cmd_ptu_pub.publish(ptu)
                 
    def callback(self, data):
        # Desired velocity in radians
        if (data.buttons[self.ptu_button] == 1):
            self.des_pan_vel = data.axes[0]*self.pan_speed
            self.des_tilt_vel = data.axes[1]*self.tilt_speed
        else:
            self.des_pan_vel = 0
            self.des_tilt_vel = 0

        if data.buttons[self.home_button] == 1:
            self.ptu_reset = True
        else:
            self.ptu_reset = False

if __name__ == "__main__": PTUTeleop()
