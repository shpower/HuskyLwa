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
import rospy
import sys, tty, termios

from sensor_msgs.msg import Joy

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        # detect ctl-c and ctl-d
        if (ord(ch) == 3 or ord(ch) == 4):
            raise KeyboardInterrupt
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class FakeJoystick:
    def __init__(self):
        rospy.init_node('fake_joystick')

        self.cmd_vel = None
        joy_pub = rospy.Publisher('/joy', Joy)

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            rate.sleep()
            joy_msg = Joy()
            k = getch()
            for i in range(0, 10):
                joy_msg.axes.append(0)
            if (k == '1'):
                joy_msg.axes[0] = 1.0
            elif (k == 'q'):
                joy_msg.axes[0] = -1.0
            elif (k == '2'):
                joy_msg.axes[1] = 1.0
            elif (k =='w'):
                joy_msg.axes[1] = -1.0
            elif (k == '3'):
                joy_msg.axes[2] = 1.0
            elif (k =='e'):
                joy_msg.axes[2] = -1.0
            elif (k == '4'):
                joy_msg.axes[3] = 1.0
            elif (k =='r'):
                joy_msg.axes[3] = -1.0                                
            elif (k == '5'):
                joy_msg.axes[4] = 1.0
            elif (k =='t'):
                joy_msg.axes[4] = -1.0
            elif (k == '6'):
                joy_msg.axes[5] = 1.0
            elif (k =='y'):
                joy_msg.axes[5] = -1.0
            elif (k == '7'):
                joy_msg.axes[6] = 1.0
            elif (k =='u'):
                joy_msg.axes[6] = -1.0
            elif (k == '8'):
                joy_msg.axes[7] = 1.0
            elif (k =='i'):
                joy_msg.axes[7] = -1.0                                                                
            elif (k == '9'):
                joy_msg.axes[8] = 1.0
            elif (k =='o'):
                joy_msg.axes[8] = -1.0
            elif (k == '0'):
                joy_msg.axes[9] = 1.0
            elif (k =='p'):
                joy_msg.axes[9] = -1.0
            joy_pub.publish(joy_msg)

if __name__ == "__main__": FakeJoystick()
