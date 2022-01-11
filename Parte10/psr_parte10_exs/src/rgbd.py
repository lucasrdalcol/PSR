#!/usr/bin/env python3
import math
from functools import partial

import rospy
import std_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs import point_cloud2

publisher = rospy.Publisher("/left_laser/pointcloud", PointCloud2, queue_size=1)


def callbackMessageReceived(msg):
    rospy.loginfo('Received PointCloud2 messages')

    pc2 = msg.data
    pc2_sample = pc2[::10]

    publisher.publish(pc2_sample)
    rospy.loginfo('Published subsampled PointCloud2 msg')


def main():
    rospy.init_node('rgbd_subscriber', anonymous=True)

    rospy.Subscriber("/tilt_scan", PointCloud2, callbackMessageReceived)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()
