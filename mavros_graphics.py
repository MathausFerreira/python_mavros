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
import time
import sys
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Image
from mavros_msgs.msg import State
import matplotlib.pyplot as plt

from cv_bridge import CvBridge, CvBridgeError
import cv2

# Check if everything was successfully imported
print ("Everything was successfully imported")


class Img_convert:    

    def __init__(self):
        self.image_sub = rospy.Subscriber('/iris/usb_cam/image_raw', Image, self.Image_Callback)

        self.bridge_object = CvBridge()

    def Image_Callback(self,data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

class threeDof:
    def __init__(self):
        self.x = []
        self.y = []
        self.z = []

class Graphplot:
    velo = threeDof()
    pose = threeDof()

    def __init__(self):
        # In ROS, nodes are uniquely named. If two nodes with the same # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        self.node = rospy.init_node('python_listener', anonymous=True)

        self.pose_sub  = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.Pose_Callback)
        self.velo_sub  = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, self.Velo_Callback)

    def plot_all(self):

        plt.subplot(2,1,1)
        plt.title('Velocity')
        plt.xlabel('Step (un)')
        plt.ylabel('Speed(m/s)')

        plt.plot(self.velo.x,'r-')
        plt.plot(self.velo.y,'g-')
        plt.plot(self.velo.z,'b-')
        plt.legend(['Vel x', 'Vel y', 'Vel z'])

        plt.subplot(2,1,2)
        plt.title('Position')
        plt.xlabel('Step (un)')
        plt.ylabel('Position(m)')

        plt.plot(self.pose.x,'r-')
        plt.plot(self.pose.y,'g-')
        plt.plot(self.pose.z,'b-')
        plt.legend(['Pos x', 'Pos y', 'Pos z'])

        plt.ion()
        plt.show()
        plt.pause(0.5)

    def Pose_Callback(self,data):
        self.pose.x.append(data.pose.position.x)
        self.pose.y.append(data.pose.position.y)
        self.pose.z.append(data.pose.position.z)
        #  if len(pose.x)> 50:
        #     self.pose.x.pop(0)
        #     self.pose.y.pop(0)
        #     self.pose.z.pop(0)

    def Velo_Callback(self,data):
        self.velo.x.append(data.twist.linear.x)
        self.velo.y.append(data.twist.linear.y)
        self.velo.z.append(data.twist.linear.z)

        # if len(velo.x)> 50:
        #     self.velo.x.pop(0)
        #     self.velo.y.pop(0)
        #     self.velo.z.pop(0)
        self.plot_all()


def main():

    # dataplot = Graphplot()
    ic = Img_convert()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        # cv2.destroyAllWindows()


if __name__ == '__main__':
    main()