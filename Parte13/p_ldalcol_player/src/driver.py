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
from geometry_msgs.msg import Twist
from tf2_geometry_msgs import PoseStamped


class Driver:
    def __init__(self):
        # Define all variables
        self.name = rospy.get_name()
        self.name = self.name.strip('/')  # remove initial /
        print('player name is ' + self.name)
        self.goal = PoseStamped()
        self.goal_active = False
        self.angle = 0
        self.speed = 0

        # Initialize publisher
        self.publisher_command = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=1)

        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Use Timer to set a Command Callback
        self.timer = rospy.Timer(rospy.Duration(0.1), self.sendCommandCallback)

        # Initialize the subscriber to receive the goal pose
        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped,
                                                self.goalReceivedCallback)  # Subscribe the node to the specified topic

    def goalReceivedCallback(self, msg):
        """

        :param msg:
        """
        print('Received new goal')
        target_frame = self.name + '/odom'

        try:
            self.goal = self.tf_buffer.transform(msg, target_frame, rospy.Duration(1))
            self.goal_active = True
            rospy.loginfo('Setting new goal on frame ' + target_frame)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.goal_active = False
            rospy.logerr('Could not transform goal from ' + msg.header.frame_id + ' to ' + target_frame +
                         '. Ignoring this goal.')

    def driveStraight(self, minimum_speed=0.1, maximum_speed=0.75):
        goal_copy = copy.deepcopy(self.goal)
        goal_copy.header.stamp = rospy.Time.now()
        goal_in_base_link = self.tf_buffer.transform(goal_copy, self.name + '/base_footprint', rospy.Duration(1))

        x = goal_in_base_link.pose.position.x
        y = goal_in_base_link.pose.position.y

        self.angle = math.atan2(y, x)

        print(self.angle)
        distance_to_goal = math.sqrt(x ** 2 + y ** 2)

        self.speed = max(minimum_speed, 0.5 * distance_to_goal)  # limit minimum speed
        self.speed = min(maximum_speed, self.speed)  # limit maximum speed

        if distance_to_goal < 0.08:
            self.speed = 0

    def sendCommandCallback(self, event):
        print('Sending twist command')

        if not self.goal_active:  # No goal, no movement
            self.speed = 0
            self.angle = 0
        else:
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
