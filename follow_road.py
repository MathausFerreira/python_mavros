import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from threading import Thread
import time
from sensor_msgs.msg import Image
import rospy


class Img_convert:    

    def __init__(self):

        self.image_sub = rospy.Subscriber('/iris/usb_cam/image_raw', Image, self.Image_Callback)
        self.image_pub = rospy.Publisher("image_opencv_topic",Image, queue_size=1)

        self.bridge_object = CvBridge()
        

    def Image_Callback(self,data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)

        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge_object.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

class Road_follow:
    
    def __init__(self):

        # self.node = rospy.init_node('python_listener', anonymous=True)

        self.image_sub = rospy.Subscriber('/image_opencv_topic', Image, self.Image_Callback)
        
        self.bridge_object = CvBridge()

        # # send setpoints in separate thread to better prevent failsafe
        # self.att_thread = Thread(target=self.run, args=())
        # # self.att_thread.daemon = True
        # self.att_thread.start()

    def Image_Callback(self,data):
        try:
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)     

        # To put a square in the center of the image        
        square_size = 20
        square_size = square_size/2

        rows, cols, _ = self.cv_image.shape

        center_x = int(cols / 2)
        center_y = int(rows / 2)      

        hsv_frame = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        # yellow color
        high_yellow  = np.array([40, 255, 255])
        low_yellow = np.array([20, 50, 50])
        yellow_mask = cv2.inRange(hsv_frame, low_yellow, high_yellow)

        cv2.rectangle(self.cv_image,(center_x - square_size,center_y - square_size),(center_x + square_size, center_y + square_size),(0,0,255),2)
        cv2.imshow("Image CV", self.cv_image)
        cv2.imshow("mask",yellow_mask)
        key = cv2.waitKey(1)

    # def run(self):
    #     global px, py
        
    #     while True:
    #         ax = False
    #         x = 0
    #         y = 0
    #         w = 0
    #         h = 0

    #         #converto from RGB to HSV
    #         hsv_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            
    #         # yellow color
    #         high_yellow  = np.array([40, 255, 255])
    #         low_yellow = np.array([20, 50, 50])
    #         yellow_mask = cv2.inRange(hsv_frame, low_yellow, high_yellow)

    #         contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #         contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
    #         for cnt in contours:
    #             (x, y, w, h) = cv2.boundingRect(cnt)
    #             # x_medium = int((x + x + w) / 2)
    #             ax = True

    #         if ax:
    #             print("Goal x :{0} Goal y: {1}".format(x,y))
    #             # cv2.line(frame, (x_medium, 0), (x_medium, 480), (0, 255, 0), 2)

    #         # cv2.rectangle(frame,(left,top),(right,down),(255,0,0),2
                     
    #     self.cv_image.release()
    #     cv2.destroyAllWindows()

if __name__ == '__main__':
    
    green = np.uint8([[[0,180,255 ]]])
    hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
    print hsv_green

    rospy.init_node('image_converter', anonymous=True)

    CVImg = Img_convert()
    N = Road_follow()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
