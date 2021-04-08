#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$


import rospy
import numpy as np
import sys
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
import matplotlib.pyplot as plt

# Check if everything was successfully imported
print ("Everything was successfully imported")

def Pose_Callback(data):
    global pose 
    pose.x.append(data.pose.position.x)
    pose.y.append(data.pose.position.y)
    pose.z.append(data.pose.position.z)

def Velo_Callback(data):
    global velo

    velo.x.append(data.twist.linear.x)
    velo.y.append(data.twist.linear.y)
    velo.z.append(data.twist.linear.z)
        
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('python_listener', anonymous=False)

    pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, Pose_Callback)
    velo_sub = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, Velo_Callback)
    # state_sub = rospy.Subscriber('/mavros/state', State, State_Callback)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

class velocity:

    def __init__(self):
        self.x = []
        self.y = []
        self.z = []
    


def init():
    global velo, pose

    velo = velocity()
    pose = velocity()

    # velo.x = []
    # velo.y = []
    # velo.z = []
    
    # pose.x = []
    # pose.y = []
    # pose.z = []

    # velo = []
    # pose = []
    

    # Thread to send setpoints
    # tSetPoints = Thread(target=listener).start()

def plot_all():
    global pose, velo

    plt.subplot(2,1,1)

    plt.title('Velocity')
    plt.xlabel('Step (un)')
    plt.ylabel('Speed(m/s)')

    plt.plot(velo.x,'r-')
    plt.plot(velo.y,'g-')
    plt.plot(velo.z,'b-')
    plt.legend(['Vel x', 'Vel y', 'Vel z'])

    plt.subplot(2,1,2)

    plt.title('Position')
    plt.xlabel('Step (un)')
    plt.ylabel('Speed(m/s)')

    plt.plot(pose.x,'r-',label = 'Pos x')
    plt.plot(pose.y,'g-',label = 'Pos y')
    plt.plot(pose.z,'b-',label = 'Pos z')
    plt.legend(['Pos x', 'Pos y', 'Pos z'])

    plt.ion()
    plt.show()
    plt.pause(0.5)



if __name__ == '__main__':
    init()
    listener()
    while True:
        plot_all()