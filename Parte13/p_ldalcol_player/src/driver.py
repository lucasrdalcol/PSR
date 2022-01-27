#!/usr/bin/env python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import argparse
import copy
import math

import colorama
import rospy
import tf2_ros
from std_msgs.msg import String
from psr_parte09_exs.msg import Dog
from geometry_msgs.msg import Twist, PoseStamped


class Driver():
    def __init__(self):
        self.publisher_command = rospy.Publisher('p_ldalcol/cmd_vel', Twist, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.sendCommandCallback)

        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped,
                                                self.goalReceivedCallback)  # Subscribe the node to the specified topic

        self.goal = PoseStamped()
        self.goal_active = False

        self.angle = 0
        self.speed = 0

    def goalReceivedCallback(self, msg):
        print('Received new goal')
        frame_id = msg.header.frame_id
        if 'odom' in frame_id:
            self.goal = copy.deepcopy(msg)
            self.goal_active = True
        else:
            rospy.logwarn('move_base_simple/goal is not in odom frame_id, aborting!!!')
            exit(0)

    def driveStraight(self):
        goal_copy = copy.deepcopy(self.goal)
        goal_copy.header.stamp = rospy.Time.now()
        goal_in_base_link = self.tf_buffer.transform(goal_copy, 'p_ldalcol/base_footprint', rospy.Duration(1))

        self.angle = math.atan2(goal_in_base_link.pose.position.y, goal_in_base_link.pose.position.x)
        self.speed = 0.1  # TODO vary speed according to distance to goal

    def sendCommandCallback(self, event):
        print('Sending twist command')

        self.driveStraight()
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.angle

        self.publisher_command.publish(twist)


def publisher():
    # ----------------------------------
    # Initialization
    # ----------------------------------
    rospy.init_node('p_ldalcol_driver', anonymous=False)  # Initialize the node
    driver = Driver()

    rate = rospy.Rate(10)  # time rate of the message

    rospy.spin()

    # ----------------------------------
    # Execution
    # ----------------------------------
    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = -1

        publisher.publish(twist)
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
