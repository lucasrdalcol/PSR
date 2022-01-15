#!/usr/bin/env python3

import math
from random import random
from functools import partial
import rospy
import std_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

publisher = rospy.Publisher("marker_array", MarkerArray, queue_size=1)


def create_marker(id):
    marker = Marker()
    marker.header.frame_id = "left_laser"
    marker.id = id
    marker.type = marker.SPHERE_LIST
    marker.action = marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.w = 1.0
    marker.color.r = random()
    marker.color.g = random()
    marker.color.b = random()
    marker.color.a = 1.0  # Don't forget to set the alpha!
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    return marker


def callbackMessageReceived(msg):
    rospy.loginfo('Received laser scan messages')

    thresh = 0.8

    marker_array = MarkerArray()
    marker = create_marker(0)
    marker_array.markers.append(marker)

    for idx, (range1, range2) in enumerate(zip(msg.ranges[:-1], msg.ranges[1:])):
        if range1 < 0.1 or range2 < 0.1:
            continue

        diff = abs(range2 - range1)
        if diff > thresh:
            marker = create_marker(idx + 1)
            marker_array.markers.append(marker)

        theta = msg.angle_min + msg.angle_increment * idx
        x = range1 * math.cos(theta)
        y = range1 * math.sin(theta)

        point = Point(x=x, y=y, z=0)
        last_marker = marker_array.markers[-1]
        last_marker.points.append(point)

    publisher.publish(marker_array)
    rospy.loginfo('Published clustered laser scan messages')


def main():
    rospy.init_node('lidar_clustering', anonymous=True)

    rospy.Subscriber("/left_laser/laserscan", LaserScan, callbackMessageReceived)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()
