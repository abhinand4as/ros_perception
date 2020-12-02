#!/usr/bin/env python

import numpy as np
import cv2
import time
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
mask_image_pub = rospy.Publisher("mask_image",Image, queue_size=10)
# rect_pts_pub = rospy.Publisher("rect_pts", Int32, queue_size=1)
bridge = CvBridge()
rect_pts = Int32()
def read_rgb_image(image_name, show):
    rgb_image = cv2.imread(image_name)
    if show: 
        cv2.imshow("RGB Image",rgb_image)
    return rgb_image

def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    #convert the image into the HSV color space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv image",hsv_image)

    #define a mask using the lower and upper bounds of the yellow color 
    mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)

    return mask


    

def getContours(binary_image):     
    #_, contours, hierarchy = cv2.findContours(binary_image, 
    #                                          cv2.RETR_TREE, 
    #                                           cv2.CHAIN_APPROX_SIMPLE)
    # contours, hierarchy = cv2.findContours(binary_image.copy(), 
    #                                         cv2.RETR_EXTERNAL,
	#                                         cv2.CHAIN_APPROX_SIMPLE)

    _, contours, _ = cv2.findContours(binary_image.copy(), 
                                            cv2.RETR_EXTERNAL,
	                                        cv2.CHAIN_APPROX_SIMPLE)
    return contours


def draw_ball_contour(binary_image, rgb_image, contours):
    # black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')
    mask_image = np.zeros(rgb_image.shape[:2],dtype="uint8")
    for c in contours:
        area = cv2.contourArea(c)
        perimeter= cv2.arcLength(c, True)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        contours_poly = cv2.approxPolyDP(c, 3, True)
        boundRect = cv2.boundingRect(contours_poly)
        
        if (area>10000):

            color = (0, 255, 0)
            cv2.drawContours(rgb_image, contours_poly, -1, color)
            cv2.rectangle(rgb_image, (int(boundRect[0]), int(boundRect[1])), \
            (int(boundRect[0]+boundRect[2]), int(boundRect[1]+boundRect[3])), color, 2)

            cv2.rectangle(mask_image, (int(boundRect[0]), int(boundRect[1])), \
            (int(boundRect[0]+boundRect[2]), int(boundRect[1]+boundRect[3])), 255, -1)

            print ("Area: {}, Perimeter: {}".format(area, perimeter))
            # rect_pts.data = [int(boundRect[0]), int(boundRect[1]), int(boundRect[0]+boundRect[2]), int(boundRect[1]+boundRect[3])]
    print ("number of contours: {}".format(len(contours)))
    cv2.imshow("RGB Image Contours",rgb_image)
    cv2.imshow("Black Image Contours",mask_image)
    return mask_image

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy

def detect_ball_in_a_frame(image_frame):
    yellowLower =(20, 100, 100)
    yellowUpper = (30, 255, 255)
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, yellowLower, yellowUpper)
    contours = getContours(binary_image_mask)
    mask_image = draw_ball_contour(binary_image_mask, rgb_image,contours)
    return mask_image


def image_callback(ros_image):
    print ('got an image')
    global bridge
    #convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    #from now on, you can work exactly like with opencv
    mask_image = detect_ball_in_a_frame(cv_image)
    time.sleep(0.033)
    cv2.waitKey(3)
    # rect_pts_ = rect_pts
    try:
      mask_image_pub.publish(bridge.cv2_to_imgmsg(mask_image, "mono8"))
    #   rect_pts_pub.publish(rect_pts_)
    except CvBridgeError as e:
      print(e)
 


def main():
    rospy.init_node('contour_detection', anonymous=True)

    # image_sub = rospy.Subscriber("/rgb/image_raw",Image, image_callback)
    image_sub = rospy.Subscriber("/usb_cam/image_raw",Image, image_callback)
    # mask_image_pub = rospy.Publisher("mask_image",Image)
    print("hii")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

    # video_capture = cv2.VideoCapture(2)
    # #video_capture = cv2.VideoCapture('video/tennis-ball-video.mp4')

    # while(True):
    #     ret, frame = video_capture.read()
    #     detect_ball_in_a_frame(frame)
    #     time.sleep(0.033)
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break

    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

