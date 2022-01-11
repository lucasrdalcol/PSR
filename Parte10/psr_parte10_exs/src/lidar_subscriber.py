#!/usr/bin/env python3
import math
from functools import partial

import rospy
import std_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs import point_cloud2

# inspiration from: https://gist.github.com/lucasw/ea04dcd65bc944daea07612314d114bb
publisher = rospy.Publisher("/left_laser/pointcloud", PointCloud2, queue_size=1)


def callbackMessageReceived(msg):
    rospy.loginfo('Received laser scan messages')

    header = std_msgs.msg.Header(seq=msg.header.seq, stamp=msg.header.stamp, frame_id=msg.header.frame_id)
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]

    # convert from polar coordinates to cartesian and fill the point cloud
    points = []
    z = 0

    for idx, range in enumerate(msg.ranges):
        theta = msg.angle_min + msg.angle_increment * idx
        x = range * math.cos(theta)
        y = range * math.sin(theta)

        points.append([x, y, z])

    pc2 = point_cloud2.create_cloud(header, fields, points)
    publisher.publish(pc2)
    rospy.loginfo('Published PointCloud2 msg')


def main():
    rospy.init_node('lidar_subscriber', anonymous=True)

    rospy.Subscriber("/left_laser/laserscan", LaserScan, callbackMessageReceived)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()
