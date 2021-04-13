import rospy
import mavros
import time
import numpy as np
from threading import Thread
from geometry_msgs.msg import PoseStamped, Point, TwistStamped,Quaternion, Vector3
from sensor_msgs.msg import Imu, Range
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Header
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import State, PositionTarget
from tf.transformations import quaternion_from_euler

class MavrosOffboardAttCtrl:

    def __init__(self):
        # Node initiation
        rospy.init_node('control_position_setpoint_py')

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

def main():

    # dataplot = Graphplot()
    Ctrl = MavrosOffboardAttCtrl()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        # cv2.destroyAllWindows()


if __name__ == '__main__':
    main()