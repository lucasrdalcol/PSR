#!/usr/bin/env python3

import math
from functools import partial
import rospy
import std_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray


def main():
    publisher = rospy.Publisher('markers', MarkerArray, queue_size=1)
    rospy.init_node('register', anonymous=True)
    rate = rospy.Rate(10)  # time rate of the message

    idx = 0
    increment = 0.1

    while not rospy.is_shutdown():

        # Initialize marker array
        markers_array = MarkerArray()

        # Sphere
        marker_sphere = Marker()
        marker_sphere.header.frame_id = "map"
        marker_sphere.ns = "my_sphere"
        marker_sphere.type = marker_sphere.SPHERE
        marker_sphere.action = marker_sphere.ADD
        marker_sphere.pose.position.x = idx
        marker_sphere.pose.position.y = 0
        marker_sphere.pose.position.z = 0
        marker_sphere.pose.orientation.x = 0.0
        marker_sphere.pose.orientation.y = 0.0
        marker_sphere.pose.orientation.z = 0.0
        marker_sphere.pose.orientation.w = 1.0
        marker_sphere.scale.x = 1
        marker_sphere.scale.y = 1
        marker_sphere.scale.z = 1
        marker_sphere.color.a = 0.5  # Don't forget to set the alpha!
        marker_sphere.color.r = 0.0
        marker_sphere.color.g = 1.0
        marker_sphere.color.b = 0.0
        markers_array.markers.append(marker_sphere)

        idx += increment
        if idx >= 3 or idx < -3:
            increment = -increment

        # Cube
        marker_cube = Marker()
        marker_cube.header.frame_id = "map"
        marker_cube.ns = "my_cube"
        marker_cube.type = marker_cube.CUBE
        marker_cube.action = marker_cube.ADD
        marker_cube.pose.position.x = 0
        marker_cube.pose.position.y = 0
        marker_cube.pose.position.z = 0
        marker_cube.pose.orientation.x = 0.0
        marker_cube.pose.orientation.y = 0.0
        marker_cube.pose.orientation.z = 0.0
        marker_cube.pose.orientation.w = 1.0
        marker_cube.scale.x = 0.5
        marker_cube.scale.y = 0.5
        marker_cube.scale.z = 0.5
        marker_cube.color.a = 1.0  # Don't forget to set the alpha!
        marker_cube.color.r = 1.0
        marker_cube.color.g = 0.0
        marker_cube.color.b = 0.0
        markers_array.markers.append(marker_cube)

        # Text
        marker_text = Marker()
        marker_text.header.frame_id = "map"
        marker_text.ns = "my_text"
        marker_text.text = 'radius = 1'
        marker_text.type = marker_text.TEXT_VIEW_FACING
        marker_text.action = marker_text.ADD
        marker_text.pose.position.x = 0
        marker_text.pose.position.y = 0.5
        marker_text.pose.position.z = 0.5
        marker_text.pose.orientation.x = 0.0
        marker_text.pose.orientation.y = 0.0
        marker_text.pose.orientation.z = 0.0
        marker_text.pose.orientation.w = 1.0
        marker_text.scale.z = 0.2
        marker_text.color.a = 1.0  # Don't forget to set the alpha!
        marker_text.color.r = 0.0
        marker_text.color.g = 0.0
        marker_text.color.b = 0.0
        markers_array.markers.append(marker_text)

        # Publish marker array
        publisher.publish(markers_array)
        rospy.loginfo('Markers are being published')

        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()
