import cv2
import rospy
import numpy as np
import time
import mavros
import math 
from cv_bridge import CvBridge, CvBridgeError
from threading import Thread
from std_msgs.msg import Header, Float64
from sensor_msgs.msg import Image
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, Point, TwistStamped,Quaternion, Vector3

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

        try:
            self.image_pub.publish(self.bridge_object.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def distAB(X,Y):
    return math.sqrt((X[0] - Y[0])**2) + ((X[1]-Y[1])**2)

def Eq_reta(P1,P2):
    x1 = float(P1[0])
    y1 = float(P1[1])
    x2 = float(P2[0])
    y2 = float(P2[1])

    if (x2-x1)!=0:
        A  = (y2-y1)/(x2-x1)
    else:
        A=0

    B = y1-A*x1

    return A,B

def Angle_retas(R1,R2):
    ms = R1
    mr = R2
    if (1+ms*mr)!=0:
        tgA = abs((ms-mr)/(1+ms*mr))
    else:
        tgA = 0.0

    Ang = math.atan(tgA)
    return Ang


class Img_command:
    
    def __init__(self):
        # self.image_sub = rospy.Subscriber('/image_opencv_topic', Image, self.Image_Callback)
        self.image_sub = rospy.Subscriber('/iris/usb_cam/image_raw', Image, self.Image_Callback)
        self.global_hdg_sub = rospy.Subscriber('mavros/global_position/compass_hdg',Float64,self.global_compass_callback)
        self.bridge_object = CvBridge()
        
        rospy.wait_for_message('/iris/usb_cam/image_raw', Image,10)
        rospy.wait_for_message('mavros/global_position/compass_hdg',Float64,10)
        
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.yaw = self.global_hdg * math.pi/180.0
        self.rate = rospy.Rate(10)

    def Img_Control(self,sqr_size=10):
        # Draw a square in the center of the image        
        sqr_size = sqr_size/2
        rows, cols, _ = self.cv_image.shape
        center_x = int(cols / 2)
        center_y = int(rows / 2)      
        cv2.rectangle(self.cv_image,(center_x - sqr_size,center_y - sqr_size),(center_x + sqr_size, center_y + sqr_size),(0,0,0),2)

        hsv_frame = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        
        # Cria uma mascara yellow color
        high_yellow  = np.array([40, 255, 255])
        low_yellow   = np.array([20, 50, 50])
        yellow_mask  = cv2.inRange(hsv_frame, low_yellow, high_yellow)

        # Encontra o contorno da mascara
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Organiza do maior para o menor
        contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        # Desenha todas as mascaras
        cv2.drawContours(self.cv_image, contours, -1, (0,255,0), 2)
        
        # Verm_x1 = 0
        # Verm_y1 = 0
        # azul_x1 = 0
        # azul_y1 = 0
        d1 = 0
        d2 = 0
        #Variavel de  folga de tamanho
        slack = 20
        #Variavel de velocidade
        stp = .75

        try:
            #Encontra os momomentos
            M1 = cv2.moments(contours[0])
            if M1['m00'] !=0:
                Verm_x1 = int(M1['m10']/M1['m00'])
                Verm_y1 = int(M1['m01']/M1['m00'])
                cv2.circle(self.cv_image,(Verm_x1,Verm_y1),1,(0,0,255),2)

            M2 = cv2.moments(contours[1])
            if M2['m00']!= 0:
                azul_x1 = int(M2['m10']/M2['m00'])
                azul_y1 = int(M2['m01']/M2['m00'])
                cv2.circle(self.cv_image,(azul_x1,azul_y1),1,(255,0,0),2)
            
            A1,B1 = Eq_reta((Verm_x1,Verm_y1),(azul_x1,azul_y1))
            A2,B2 = Eq_reta((center_x-100,center_y),(center_x+100,center_y))

            # represents the top left corner of image
            start_point = (Verm_x1,Verm_y1)        
            # represents the bottom right corner of image
            end_point = (azul_x1,azul_y1)
            # Yellow color in BGR
            color = (0, 255, 255)
            # Draw a diagonal green line with thickness of 9 px
            cv2.line(self.cv_image, start_point, end_point, color,thickness=1)
            
            # represents the top left corner of image
            start_point = (center_x-100,center_y)
            # represents the bottom right corner of image
            end_point = (center_x+100,center_y)
            # White color in BGR
            color = (255, 255, 255)
            cv2.line(self.cv_image, start_point, end_point, color,thickness=1)

            Angle = Angle_retas(A1,A2)
            # XAS  =Angle*(180.0/np.pi)

            # print("Angle : {0}".format(XAS))
            # print("Angle: {0}  A1:{1}  A2:{2}".format(Angle,A1,A2))


            if( Verm_x1>azul_x1):
                d1 = distAB((Verm_x1,0), (self.center_x,0))
                d2 = distAB((self.center_x,0), (azul_x1,0))
            elif azul_x1>Verm_x1:
                d1 = distAB((azul_x1,0),(self.center_x,0))
                d2 = distAB((self.center_x,0),(Verm_x1,0))
            else:
                d1 = 0
                d2 = 0
            
            # self.yaw = np.float32(self.Zero + Angle*(180.0/np.pi))
            # print("YAW: {0}".format(self.yaw))
        except:
            pass

        if d1 > d2+slack:
            self.vy = -1*stp
            # print("direita")
        elif d2 > d1+slack:
            self.vy = stp
            # print("esquerda")
        else:
            self.vy = 0.0

        self.vx = 2.0

        self.yaw = ((0.0 * math.pi/180.0)+(self.global_hdg * math.pi/180.0) + 0.5*math.atan2((Verm_y1-azul_y1),(Verm_x1-azul_x1)))%360.0
        
        H = self.yaw*(180/math.pi)

        print("YAW: {0}".format(H))
        
        cv2.imshow("Image CV", self.cv_image)      # cv2.imshow("mask",yellow_mask)
        key = cv2.waitKey(1)

    def Image_Callback(self,data):
        try:
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)     

        self.Img_Control()
        try:  # prevent garbage in console output when thread is killed
            self.rate.sleep()
        except:
            pass
    
    def global_compass_callback(self,data):
        self.global_hdg = data.data
        # print("Val > {0}".format(self.global_hdg))