#!/usr/bin/env python
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import sys

def improve(mask):
    # cv2.imshow("mask_raw", mask)
    kernel = np.ones([5, 5], np.uint8)
    mask = cv2.erode(mask, kernel, iterations = 1)
    mask = cv2.dilate(mask, kernel, iterations = 1)
    # cv2.imshow("mask", mask)
    return mask

def crop_mask(mask,search_top,search_bot,h,w):
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    return mask

def find_central(M,image, w,color):
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    cv2.circle(image, (cx, cy), 20, color, -1)

    err = cx - w/2
    return err

def image_callback(msg):

    ######### Image Processing ###########
    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    # blur image to reduce the noise
    image = cv2.GaussianBlur(image,(13,13),0)

    # equalize RGB channels
    image[:,:,2] = cv2.equalizeHist(image[:,:,2])
    image[:,:,1] = cv2.equalizeHist(image[:,:,1])
    image[:,:,0] = cv2.equalizeHist(image[:,:,0])

    # find white color from image]
    lower_white = np.array([250, 250, 250])
    upper_white = np.array([255, 255, 255])
    mask_white = cv2.inRange(image, lower_white, upper_white)

    # find yellow color from image
    lower_yellow = np.array([0, 230, 130])
    upper_yellow = np.array([165, 255, 255])
    mask_yellow = cv2.inRange(image, lower_yellow, upper_yellow)

    # improve yellow mask
    mask_yellow = improve(mask_yellow)

    ###### Image Processing (END) ########

    ############## Control ###############
    # refine useful area
    h, w, d = image.shape
    search_top = 2*h/16
    search_bot = 10*h/16
    mask_white = crop_mask(mask_white,search_top,search_bot,h,w)
    mask_yellow = crop_mask(mask_yellow,search_top,search_bot,h,w)

    # use moment to find error to the center
    M_white = cv2.moments(mask_white)
    M_yellow = cv2.moments(mask_yellow)

    if M_white['m00'] > 10000:
        err_white = find_central(M_white,image, w, (0,255,255)) - 180
    else:
        err_white = 0

    if M_yellow['m00'] > 200000:
        err_yellow = find_central(M_yellow,image, w, (255,0,0)) + 400
    else:
        err_yellow = 0

    # send command to robot based on the err
    err = err_white
    if(err_yellow!=0):
        err = err_yellow

    twist.linear.x = 0.6
    twist.angular.z = -float(err) / 190
    cmd_vel_pub.publish(twist)

    ########### Control (END) ############

    # visualization for human
    cv2.imshow("window", image)
    cv2.imshow("mask_white", mask_white)
    cv2.imshow("mask_yellow", mask_yellow)
    # cv2.waitKey(3)



if __name__ == '__main__':
    bridge = cv_bridge.CvBridge()
    rospy.init_node('follower')
    image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                 Image, image_callback)
    cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    twist = Twist()


    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if cv2.waitKey(3) & 0xFF == ord('q'):
            break

        rate.sleep()

    # bridge.release()
    cv2.destroyAllWindows()
