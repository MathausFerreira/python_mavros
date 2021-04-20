from __future__ import division

import rospy
import unittest
import mavros
import time
import tf
import numpy as np
import math
import unittest

from threading import Thread
from geometry_msgs.msg import PoseStamped, Point, TwistStamped, Quaternion, Vector3
from sensor_msgs.msg import Imu, Range
from mavros_msgs.srv import SetMode, CommandBool,ParamGet, SetMode, WaypointClear, WaypointPush
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, AttitudeTarget, PositionTarget
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler

from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix, Imu
from six.moves import xrange

class MavrosCommon():

    def __init__(self, *args):
        # super(MavrosCommon, self).__init__(*args)
        self.setUp()
    
    def setUp(self):
        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.imu_data = Imu()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.mission_wp = WaypointList()
        self.state = State()
        self.mav_type = None

        self.sub_topics_ready = {
            key: False
            for key in [
                'alt', 'ext_state', 'global_pos', 'home_pos', 'local_pos',
                'mission_wp', 'state', 'imu'
            ]
        }

        # ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            self.fail("failed to connect to services")

        self.get_param_srv  = rospy.ServiceProxy('mavros/param/get', ParamGet)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_srv   = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.wp_clear_srv   = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
        self.wp_push_srv    = rospy.ServiceProxy('mavros/mission/push', WaypointPush)

        # ROS subscribers
        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude, self.altitude_callback)
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state',ExtendedState,self.extended_state_callback)
        self.global_pos_sub = rospy.Subscriber('mavros/global_position/global',NavSatFix,self.global_position_callback)
        self.imu_data_sub = rospy.Subscriber('mavros/imu/data',Imu, self.imu_data_callback)
        self.home_pos_sub = rospy.Subscriber('mavros/home_position/home', HomePosition, self.home_position_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self.mission_wp_sub = rospy.Subscriber('mavros/mission/waypoints', WaypointList, self.mission_wp_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
    
    #
    # Callback functions
    #
    def altitude_callback(self, data):
        self.altitude = data

        # amsl has been observed to be nan while other fields are valid
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True

    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE']
                [self.extended_state.vtol_state].name, mavutil.mavlink.enums[
                    'MAV_VTOL_STATE'][data.vtol_state].name))

        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_LANDED_STATE']
                [self.extended_state.landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][data.landed_state].name))

        self.extended_state = data

        if not self.sub_topics_ready['ext_state']:
            self.sub_topics_ready['ext_state'] = True

    def global_position_callback(self, data):
        self.global_position = data

        if not self.sub_topics_ready['global_pos']:
            self.sub_topics_ready['global_pos'] = True

    def imu_data_callback(self, data):
        self.imu_data = data

        if not self.sub_topics_ready['imu']:
            self.sub_topics_ready['imu'] = True

    def home_position_callback(self, data):
        self.home_position = data

        if not self.sub_topics_ready['home_pos']:
            self.sub_topics_ready['home_pos'] = True

    def local_position_callback(self, data):
        self.local_position = data

        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    def mission_wp_callback(self, data):
        if self.mission_wp.current_seq != data.current_seq:
            rospy.loginfo("current mission waypoint sequence updated: {0}".
                          format(data.current_seq))

        self.mission_wp = data

        if not self.sub_topics_ready['mission_wp']:
            self.sub_topics_ready['mission_wp'] = True

    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(
                self.state.connected, data.connected))

        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.state.mode, data.mode))

        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][data.system_status].name))

        self.state = data

        # mavros publishes a disconnected state message on init
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    
    #
    # Helper methods
    #
    def set_arm(self, arm, timeout):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        old_arm = self.state.armed
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        arm_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set = True
                rospy.loginfo("set arm success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # self.assertTrue(arm_set, (
        #     "failed to set arm | new arm: {0}, old arm: {1} | timeout(seconds): {2}".
        #     format(arm, old_arm, timeout)))

    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        old_mode = self.state.mode
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        mode_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # self.assertTrue(mode_set, (
        #     "failed to set mode | new mode: {0}, old mode: {1} | timeout(seconds): {2}".
        #     format(mode, old_mode, timeout)))

    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = False
        for i in xrange(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready = True
                rospy.loginfo("simulation topics ready | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # self.assertTrue(simulation_ready, (
        #     "failed to hear from all subscribed simulation topics | topic ready flags: {0} | timeout(seconds): {1}".
        #     format(self.sub_topics_ready, timeout)))

    def wait_for_landed_state(self, desired_landed_state, timeout, index):
        rospy.loginfo("waiting for landed state | state: {0}, index: {1}".
                      format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                          desired_landed_state].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        landed_state_confirmed = False
        for i in xrange(timeout * loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed = True
                rospy.loginfo("landed state confirmed | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # self.assertTrue(landed_state_confirmed, (
        #     "landed state not detected | desired: {0}, current: {1} | index: {2}, timeout(seconds): {3}".
        #     format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
        #         desired_landed_state].name, mavutil.mavlink.enums[
        #             'MAV_LANDED_STATE'][self.extended_state.landed_state].name,
        #            index, timeout)))

    def wait_for_vtol_state(self, transition, timeout, index):
        """Wait for VTOL transition, timeout(int): seconds"""
        rospy.loginfo(
            "waiting for VTOL transition | transition: {0}, index: {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE'][
                    transition].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        transitioned = False
        for i in xrange(timeout * loop_freq):
            if transition == self.extended_state.vtol_state:
                rospy.loginfo("transitioned | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                transitioned = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # self.assertTrue(transitioned, (
        #     "transition not detected | desired: {0}, current: {1} | index: {2} timeout(seconds): {3}".
        #     format(mavutil.mavlink.enums['MAV_VTOL_STATE'][transition].name,
        #            mavutil.mavlink.enums['MAV_VTOL_STATE'][
        #                self.extended_state.vtol_state].name, index, timeout)))

    def clear_wps(self, timeout):
        """timeout(int): seconds"""
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        wps_cleared = False
        for i in xrange(timeout * loop_freq):
            if not self.mission_wp.waypoints:
                wps_cleared = True
                rospy.loginfo("clear waypoints success | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.wp_clear_srv()
                    if not res.success:
                        rospy.logerr("failed to send waypoint clear command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # self.assertTrue(wps_cleared, (
        #     "failed to clear waypoints | timeout(seconds): {0}".format(timeout)
        # ))

    def send_wps(self, waypoints, timeout):
        """waypoints, timeout(int): seconds"""
        rospy.loginfo("sending mission waypoints")
        if self.mission_wp.waypoints:
            rospy.loginfo("FCU already has mission waypoints")

        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        wps_sent = False
        wps_verified = False
        for i in xrange(timeout * loop_freq):
            if not wps_sent:
                try:
                    res = self.wp_push_srv(start_index=0, waypoints=waypoints)
                    wps_sent = res.success
                    if wps_sent:
                        rospy.loginfo("waypoints successfully transferred")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            else:
                if len(waypoints) == len(self.mission_wp.waypoints):
                    rospy.loginfo("number of waypoints transferred: {0}".
                                  format(len(waypoints)))
                    wps_verified = True

            if wps_sent and wps_verified:
                rospy.loginfo("send waypoints success | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # self.assertTrue((
        #     wps_sent and wps_verified
        # ), "mission could not be transferred and verified | timeout(seconds): {0}".
        #                 format(timeout))

    def wait_for_mav_type(self, timeout):
        """Wait for MAV_TYPE parameter, timeout(int): seconds"""
        rospy.loginfo("waiting for MAV_TYPE")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        res = False
        for i in xrange(timeout * loop_freq):
            try:
                res = self.get_param_srv('MAV_TYPE')
                if res.success:
                    self.mav_type = res.value.integer
                    rospy.loginfo(
                        "MAV_TYPE received | type: {0} | seconds: {1} of {2}".
                        format(mavutil.mavlink.enums['MAV_TYPE'][self.mav_type]
                               .name, i / loop_freq, timeout))
                    break
            except rospy.ServiceException as e:
                rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # self.assertTrue(res.success, (
        #     "MAV_TYPE param get failed | timeout(seconds): {0}".format(timeout)
        # ))


class MavrosOffboardAttCtrl:

    def __init__(self):
        # Node initiation
        # rospy.init_node('control_velocity_setpoint_py')

        self.att = AttitudeTarget()
        self.att.body_rate = Vector3()

        time.sleep(1)
        self.att_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

        rospy.wait_for_service('mavros/set_mode')
        set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        rospy.wait_for_service('mavros/cmd/arming')
        arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

        # send setpoints in separate thread to better prevent failsafe
        self.att_thread = Thread(target=self.send_att, args=())
        self.att_thread.daemon = True
        self.att_thread.start()

        self.rate = rospy.Rate(10)  # Hz
        self.att.header = Header()
        self.att.header.frame_id = "base_footprint"
        self.att.body_rate.x = 0
        self.att.body_rate.y = 0
        self.att.body_rate.z = 0

        arming_client(True)
        time.sleep(5)
        set_mode_client(custom_mode="OFFBOARD")

    def setcommand(self, x, y, z):
        self.att.body_rate.x = x
        self.att.body_rate.y = y
        self.att.body_rate.z = z

    def send_att(self):
        self.att.orientation = Quaternion(*quaternion_from_euler(0, 10, 90))
        self.att.thrust = 0.71

        self.att.type_mask = 4

        while not rospy.is_shutdown():
            self.att.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(self.att)
            try:  # prevent garbage in console output when thread is killed
                self.rate.sleep()
            except rospy.ROSInterruptException:
                pass

class MavrosVelCtrl():


    def __init__(self):
        # super(MavrosVelCtrl,self).setUp()
        # Node initiation
        # rospy.init_node('control_speed_setpoint')
        
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.yaw = 0

        self.Vel = PositionTarget()
        self.Vel_setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # self.local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_velo_callback)
        self.rate = rospy.Rate(20)
        # rospy.wait_for_service('mavros/set_mode')
        # self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        # rospy.wait_for_service('mavros/cmd/arming')
        # arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

        # arming_client(True)
        # time.sleep(5)
        # self.set_mode_client(custom_mode="OFFBOARD")

        # send setpoints in separate thread to better prevent failsafe
        self.Vel_thread = Thread(target=self.sendSetpoint, args=())
        self.Vel_thread.daemon = True
        try:
            self.Vel_thread.start()
        except:
            print("Erro ao iniciar a Thread")


    def sendSetpoint(self):
        self.Vel.header = Header()
        self.Vel.header.frame_id = "body"
        self.Vel.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.Vel.type_mask = (PositionTarget.IGNORE_AFX|
        PositionTarget.IGNORE_AFY|
        PositionTarget.IGNORE_AFZ|
        PositionTarget.IGNORE_PX|
        PositionTarget.IGNORE_PY|
        PositionTarget.IGNORE_PZ|
        PositionTarget.IGNORE_YAW_RATE|
        PositionTarget.IGNORE_YAW|
        PositionTarget.FORCE)

        while not rospy.is_shutdown():

            self.Vel.header.stamp = rospy.Time.now()

            self.Vel.velocity.x = self.vx
            self.Vel.velocity.y = self.vy
            self.Vel.velocity.z = self.vz
            self.Vel.yaw = self.yaw

            self.Vel_setpoint_pub.publish(self.Vel)

            # print("IMG CLASS Vel x : {0}   Vel y: {1}   Vel z: {2}   Yaw: {3}".format(self.vx,self.vy,self.vz,self.yaw))

            try:  # prevent garbage in console output when thread is killed
                self.rate.sleep()
            except rospy.ROSInterruptException:
                pass

    
    def local_velo_callback(self, data):
        self.local_position = data

    def setVel(self, vx,vy,vz,yaw):
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.yaw = yaw

class MavrosOffboardPosCtrl:

    # Returns a radian from a degree
    def deg2radf(self, a):
        return float(a) * pi / 180.0

    def local_position_callback(self, data):
        self.local_position = data

    def __init__(self):
        # Node initiation
        rospy.init_node('control_position_setpoint_py')
        self.setpoint = Vector3()
        self.setpoint.x = 0
        self.setpoint.y = 0
        self.setpoint.z = 0

        self.Pos = PoseStamped()

        self.local_setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=0)
        self.local_pos_sub = rospy.Subscriber(
            'mavros/local_position/pose', PoseStamped, self.local_position_callback)
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

    def sendSetpoint(self):
        
        while not rospy.is_shutdown():
            q = tf.transformations.quaternion_from_euler(
                0, 0, self.deg2radf(0.0), axes="sxyz")

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
            if self.is_at_position(self.setpoint.x, self.setpoint.y, self.setpoint.z, 1):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)

