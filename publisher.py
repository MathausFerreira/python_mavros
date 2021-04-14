import rospy
import unittest
import mavros
import time
import tf
import numpy as np
from threading import Thread
from geometry_msgs.msg import PoseStamped, Point, TwistStamped,Quaternion, Vector3
from sensor_msgs.msg import Imu, Range
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Header
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import State, PositionTarget
from tf.transformations import quaternion_from_euler
from math import pi
from opencv_track_object import Img_process

class MavrosOffboardAttCtrl:

    def __init__(self):
        # Node initiation
        rospy.init_node('control_velocity_setpoint_py')

        self.att = AttitudeTarget()

        time.sleep(1)
        self.att_setpoint_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

        rospy.wait_for_service('mavros/set_mode')
        set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        rospy.wait_for_service('mavros/cmd/arming')
        arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

        # send setpoints in separate thread to better prevent failsafe
        self.att_thread = Thread(target=self.send_att, args=())
        self.att_thread.daemon = True
        self.att_thread.start()

        arming_client(True)
        time.sleep(5)
        set_mode_client(custom_mode="OFFBOARD")

    def send_att(self):
        rate = rospy.Rate(10)  # Hz
        self.att.body_rate = Vector3()
        self.att.header = Header()
        self.att.header.frame_id = "base_footprint"
        self.att.orientation = Quaternion(*quaternion_from_euler(0,0.1,90))
        self.att.thrust = 0.76

        self.att.type_mask = 4 

        self.att.body_rate.x = 0.5
        self.att.body_rate.y = 0
        self.att.body_rate.z = 0

        while not rospy.is_shutdown():
            self.att.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(self.att)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

class MavrosOffboardPosCtrl:

    def local_position_callback(self, data):
        self.local_position = data

    def __init__(self):
        # Node initiation
        rospy.init_node('control_position_setpoint_py')
        self.setpoint = Point()
        self.setpoint.x = 0
        self.setpoint.y = 0
        self.setpoint.z = 0

        self.Pos = PoseStamped()

        self.local_setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=0)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',PoseStamped,self.local_position_callback)
        self.rate = rospy.Rate(10)
        rospy.wait_for_service('mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        rospy.wait_for_service('mavros/cmd/arming')
        arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

        arming_client(True)
        time.sleep(5)
        self.set_mode_client(custom_mode="OFFBOARD")

        # send setpoints in separate thread to better prevent failsafe
        self.Pos_thread = Thread(target=self.sendSetpoint, args=())
        self.Pos_thread.daemon = True
        self.Pos_thread.start()
    
    # Returns a radian from a degree
    def deg2radf(self,a):
        return float(a) * pi / 180.0

    def sendSetpoint(self):

        while not rospy.is_shutdown():
            q = tf.transformations.quaternion_from_euler(0, 0, self.deg2radf(0.0), axes="sxyz")

            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = float(self.setpoint.x)
            msg.pose.position.y = float(self.setpoint.y)
            msg.pose.position.z = float(self.setpoint.z)
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            self.local_setpoint_pub.publish(msg)
            try:  # prevent garbage in console output when thread is killed
                self.rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug("current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset


    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.setpoint.x = x
        self.setpoint.y = y
        self.setpoint.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        # yaw_degrees = 0  # North
        # yaw = math.radians(yaw_degrees)
        # quaternion = quaternion_from_euler(0, 0, yaw)
        # self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.setpoint.x,self.setpoint.y,self.setpoint.z, 1):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)

def main():

    # dataplot = Graphplot()
    Ctrl = MavrosOffboardPosCtrl()

    CVImg = Img_process()


    # positions = ((1, 1, 1), (50, 50, 20), (6, -50, 2), (-50, -50, 20),(0, 0, 20))

    # for i in xrange(len(positions)):
    while True:
        Ctrl.reach_position(CVImg.px,CVImg.py, 10, 30)


if __name__ == '__main__':
    main()
