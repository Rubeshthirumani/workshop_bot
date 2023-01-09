#!/usr/bin/env python3
import numpy as np
import cv2
import imutils
import rospy
import time
#speed of the motion in Secounds

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
 
def color_detector():
    CM_TO_PIXEL = 45.0 / 640

    (x, y) = (0, 0)

    cap = cv2.VideoCapture(2) 

    while not rospy.is_shutdown():   
        (ret, frame) = cap.read()

        frame = imutils.resize(frame, width=640)

        # Convert the frame in
        # BGR(RGB color space) to
        # HSV(hue-saturation-value)
        # color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Set range for red color and
        # define mask
        red_lower = np.array([136, 87, 111], np.uint8)
        red_upper = np.array([180, 255, 255], np.uint8)
        red_mask = cv2.inRange(hsv, red_lower, red_upper)

        # Set range for green color and
        # define mask
        green_lower = np.array([25, 52, 72], np.uint8)
        green_upper = np.array([102, 255, 255], np.uint8)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        # Set range for blue color and
        # define mask
        blue_lower = np.array([94, 80, 2], np.uint8)
        blue_upper = np.array([120, 255, 255], np.uint8)
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
        
        # Morphological Transform, Dilation
        # for each color and bitwise_and operator
        # between frame and mask determines
        # to detect only that particular color
        kernal = np.ones((1,1), "uint8")
        
        # For red color
        red_mask = cv2.dilate(red_mask, kernal)
        res_red = cv2.bitwise_and(frame, frame,
                                mask = red_mask)
        
        # For green color
        green_mask = cv2.dilate(green_mask, kernal)
        res_green = cv2.bitwise_and(frame, frame,
                                    mask = green_mask)
        
        # For blue color
        blue_mask = cv2.dilate(blue_mask, kernal)
        res_blue = cv2.bitwise_and(frame, frame,
                                mask = blue_mask)

        # Creating contour to track red color
        contours, hierarchy = cv2.findContours(red_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_NONE)
        
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 200):
                cv2.drawContours(frame, [contour], -1, (0, 0, 255), 2)
                M = cv2.moments(contour)
                if M['m00'] != 0.0:
                    x1 = int(M['m10'] / M['m00'])
                    y1 = int(M['m01'] / M['m00'])
                    x = x1 * CM_TO_PIXEL
                    y = y1 * CM_TO_PIXEL

                cv2.putText(frame, str(int(x)) + ' ' + str(int(y)), (int(x1), int(y1)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)   

        # Creating contour to track green color
        contours, hierarchy = cv2.findContours(green_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
        
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 200):
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                M = cv2.moments(contour)
                if M['m00'] != 0.0:
                    x1 = int(M['m10'] / M['m00'])
                    y1 = int(M['m01'] / M['m00'])
                    x = x1 * CM_TO_PIXEL
                    y = y1 * CM_TO_PIXEL

                cv2.putText(frame, str(int(x)) + ' ' + str(int(y)), (int(x1), int(y1)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255, 0), 2)

        # Creating contour to track blue color
        contours, hierarchy = cv2.findContours(blue_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 200):
                cv2.drawContours(frame, [contour], -1, (255, 0, 0), 2)
                M = cv2.moments(contour)
                if M['m00'] != 0.0:
                    x1 = int(M['m10'] / M['m00'])
                    y1 = int(M['m01'] / M['m00'])
                    x = x1 * CM_TO_PIXEL
                    y = y1 * CM_TO_PIXEL
                    z = 30
                cv2.putText(frame, str(int(x)) + ' ' + str(int(y)), (int(x1), int(y1)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                publish = Float32MultiArray()
                publish.layout = MultiArrayLayout()
                publish.data = [int(x),int(y),int(z)]         
                pub.publish(publish)
        # Program Termination
        cv2.imshow("Multiple Color Detection in Real-Time", frame)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break

            
if __name__ == '__main__':
    
    rospy.init_node('camera_coordinate_publisher', anonymous=True)
    pub = rospy.Publisher('/position_x_y', Float32MultiArray, queue_size=2)
    
    try:
     print("Camera_coordinate publisher initiated now px - 640 x 480")  

     color_detector()
     
    except rospy.ROSInterruptException:
        print ("EXCEPTION")


    