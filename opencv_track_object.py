import cv2
import numpy as np
from threading import Thread
import time

class Img_process:
    
    
    def __init__(self):
        self.py = 0
        self.px = 0
        
        self.goal_x = 0
        self.goal_y = 0

        self.cap = cv2.VideoCapture(0)

        self.cap.set(3, 480)
        self.cap.set(4, 320)

        #Finding the centor of the image
        _, frame = self.cap.read()
        rows, cols, _ = frame.shape

        self.center_x = int(cols / 2)
        self.center_y = int(rows / 2)
        # No attitude area
        square_size = 100
        square_size = square_size/2

        self.top   = self.center_y-square_size
        self.down  = self.center_y+square_size
        self.left  = self.center_x-square_size
        self.right = self.center_x+square_size

        # send setpoints in separate thread to better prevent failsafe
        self.att_thread = Thread(target=self.run, args=())
        # self.att_thread.daemon = True
        self.att_thread.start()


    def printgoal(self):
        print("goal_x: {0} Goal_y: {1} ".format(self.goal_x,self.goal_y))

    def printSP(self):
        print("SP_x: {0} SP_y: {1} ".format(self.px,self.py))

    def newSP(self):
        stp = 0.1
        if self.goal_x > self.right:
            self.py += stp
        elif self.goal_x < self.left:
            self.py -= stp

        if self.goal_y < self.top:
            self.px += stp
        elif self.goal_y >self.down:
            self.px -= stp

        self.printSP()

    def run(self):
        global px, py
        
        while True:
            _, frame = self.cap.read()
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # red color
            low_red  = np.array([161, 155, 84])
            high_red = np.array([179, 255, 255])

            red_mask = cv2.inRange(hsv_frame, low_red, high_red)

            contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
            
            ax = False
            x = 0
            y = 0
            w = 0
            h = 0
            for cnt in contours:
                (x, y, w, h) = cv2.boundingRect(cnt)
                # x_medium = int((x + x + w) / 2)
                ax = True
                break

            if ax:
                cv2.rectangle(frame,(x,y),(x + w, y + h),(0,0,255),2)
                # cv2.line(frame, (x_medium, 0), (x_medium, 480), (0, 255, 0), 2)
            
            self.goal_x = x + w/2
            self.goal_y = y + w/2

            # cv2.rectangle(frame,(left,top),(right,down),(255,0,0),2)
            color = (255,125,0)
            cv2.line(frame, (0,self.top), (480,self.top), (0,0,0), 2)
            cv2.line(frame, (0,self.down), (480,self.down), color, 2)
            cv2.line(frame, (self.right, 0), (self.right,480), color, 2)
            cv2.line(frame, (self.left,0), (self.left,480), color, 2)
            self.newSP()

            cv2.imshow("Frame",frame) # cv2.imshow("mask",red_mask)
            
            key = cv2.waitKey(1)
            
        
            # # Move servo motor
            # if x_medium < center -30:
            #     position += 1.5
            # elif x_medium > center + 30:
            #     position -= 1.5
                
            # pwm.setServoPosition(0, position)
            
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    CVImg = Img_process()

    while True:
        CVImg.printgoal()
        # print("goal_x: {0} Goal_y: {1} ".format(CVImg.goal_x,CVImg.goal_y))