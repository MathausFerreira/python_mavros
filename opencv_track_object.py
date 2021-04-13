import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(3, 480)
cap.set(4, 320)

#Finding the centor of the image
_, frame = cap.read()
rows, cols, _ = frame.shape

center_x = int(cols / 2)
center_y = int(rows / 2)

while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # red color
    low_red  = np.array([161, 155, 84])
    high_red = np.array([179, 255, 255])

    red_mask = cv2.inRange(hsv_frame, low_red, high_red)

    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
    
    ax = False
    for cnt in contours:
        (x, y, w, h) = cv2.boundingRect(cnt)
        
        # x_medium = int((x + x + w) / 2)
        ax = True
        break

    if ax:
        cv2.rectangle(frame,(x,y),(x + w, y + h),(0,0,255),2)
        # cv2.line(frame, (x_medium, 0), (x_medium, 480), (0, 255, 0), 2)
    
    # No attitude area
    square_size = 100
    square_size = square_size/2

    top = center_y-square_size
    down = center_y+square_size
    left = center_x-square_size
    right = center_x+square_size

    # cv2.rectangle(frame,(left,top),(right,down),(255,0,0),2)
    color = (255,125,0)
    cv2.line(frame, (0,top), (480,top), color, 2)
    cv2.line(frame, (0,down), (480,down), color, 2)
    cv2.line(frame, (right, 0 ), (right,480), color, 2)
    cv2.line(frame, (left,0), (left,480), color, 2)

    cv2.imshow("Frame",frame)
    # cv2.imshow("mask",red_mask)
    
    key = cv2.waitKey(1)
    
    if key == 27:
        break
    
    # # Move servo motor
    # if x_medium < center -30:
    #     position += 1.5
    # elif x_medium > center + 30:
    #     position -= 1.5
        
    # pwm.setServoPosition(0, position)
    
cap.release()
cv2.destroyAllWindows()