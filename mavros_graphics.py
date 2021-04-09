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
    plot_all()

class Img_convert:    
    def __init__(self):
        # rospy.init_node('image_converter', anonymous=True)
        # self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.image_sub = rospy.Subscriber('/iris/usb_cam/image_raw', Image, self.Image_Callback)
        self.bridge_object = CvBridge()

    def Image_Callback(self,data):
        global cv_image
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)

        # (rows,cols,channels) = cv_image.shape
        # if cols > 60 and rows > 60 :
        #     cv2.circle(cv_image, (50,50), 10, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        # try:
        #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,'8UC3'))
        # except CvBridgeError as e:
        #     print(e)

        # cv2.imshow("Camera",cv_image)
        # time.sleep(10)
    
class velocity:

    def __init__(self):
        self.x = []
        self.y = []
        self.z = []
        
def listener():
    init()

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('python_listener', anonymous=True)

    pose_sub  = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, Pose_Callback)
    velo_sub  = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, Velo_Callback)
    # state_sub = rospy.Subscriber('/mavros/state', State, State_Callback)
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
   
def init():
    global velo, pose

    velo = velocity()
    pose = velocity()

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

def main():
    ic = Img_convert()
    listener()
    # while True:
    #     plot_all()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

    # init()
    # while True:
    #     print("Running")
    #     plot_all()