#!/usr/bin/env python3
import math
from functools import partial

import cv2
import rospy
import std_msgs.msg
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Image
from sensor_msgs import point_cloud2


def main():
    # Initialize the ros node
    rospy.init_node('image_publisher', anonymous=True)

    # Configure the publisher node
    publisher = rospy.Publisher('~image', Image, queue_size=1)
    rate = rospy.Rate(15)

    # Video capture setup
    # initial setup
    capture = cv2.VideoCapture(0)  # Selecting the camera 0
    window_name = 'OpenCV Window'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # Continuously show the webcam image
    while True:
        _, frame = capture.read()  # get an image from the camera (a frame)
        # Show image
        cv2.imshow(window_name, frame)

        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        publisher.publish(image_message)

        # key to break the cycle
        key = cv2.waitKey(1)
        if key == ord('q'):
            print('You pressed q, exiting the program')
            break

        rate.sleep()

    # Release everything if job is finished
    capture.release()
    cv2.destroyAllWindows()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()
