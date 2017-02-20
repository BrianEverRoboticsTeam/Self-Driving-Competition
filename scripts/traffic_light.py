#!/usr/bin/env python
import rospy, cv2
import numpy as np
from std_msgs.msg import String

def isRightLight(cam):
    ret, frame = cam.read()
    if not ret:
        return False

    frame = cv2.GaussianBlur(frame, (35,35), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = np.array([170, 50, 150])
    upper_red = np.array([180, 255, 255])
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    cv2.imshow('red', red_mask)

    red_count = cv2.countNonZero(red_mask)
    # print 'red_count:', red_count
    return red_count > 5000

if __name__ == '__main__':
    rospy.init_node('Traffic_light')
    light_pub = rospy.Publisher('traffic_light', String, queue_size=1)
    cam = cv2.VideoCapture(1)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if isRightLight(cam):
            light_pub.publish('Stop')
        else:
            light_pub.publish('Go')

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        key = None

        rate.sleep()

    cam.release()
    cv2.destroyAllWindows()
