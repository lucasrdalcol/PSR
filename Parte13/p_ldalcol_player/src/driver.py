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

        # Initialize the subscriber to receive the goal pose with a callback
        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped,
                                                self.goalReceivedCallback)  # Subscribe the node to the specified topic

    def goalReceivedCallback(self, msg):
        """
        callback function that receives a goal pose and transform to odom frame.
        :param msg: goal pose. type: PoseStamped
        """
        print('Received new goal')
        target_frame = self.name + '/odom'  # odom frame is the target frame, always

        # Transform the goal pose to odom frame and active it, if it is not possible, ignore the goal.
        try:
            self.goal = self.tf_buffer.transform(msg, target_frame, rospy.Duration(1))
            self.goal_active = True
            rospy.loginfo('Setting new goal on frame ' + target_frame)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.goal_active = False
            rospy.logerr('Could not transform goal from ' + msg.header.frame_id + ' to ' + target_frame +
                         '. Ignoring this goal.')

    def driveStraight(self, minimum_speed=0.1, maximum_speed=0.75):
        """
        Function that receives the goal pose and calculate the speed and angle to drive to that goal.
        :param minimum_speed: minimum speed allowed to the robot when driving to the goal pose.
        :param maximum_speed: maximum speed allowed to the robot when driving to the goal pose.
        """

        # Receive the goal pose and transform to the base_footprint frame
        goal_copy = copy.deepcopy(self.goal)
        goal_copy.header.stamp = rospy.Time.now()
        goal_in_base_link = self.tf_buffer.transform(goal_copy, self.name + '/base_footprint', rospy.Duration(1))

        # Get x and y coordinates of the goal pose
        x = goal_in_base_link.pose.position.x
        y = goal_in_base_link.pose.position.y

        # Compute the angle needed to the robot goes to the goal pose.
        self.angle = math.atan2(y, x)

        # Calculate distance to goal and vary the speed according to it. Speed to go to the goal pose.
        distance_to_goal = math.sqrt(x ** 2 + y ** 2)
        self.speed = max(minimum_speed, 0.5 * distance_to_goal)  # limit minimum speed
        self.speed = min(maximum_speed, self.speed)  # limit maximum speed

        # if the robot is close enough to the goal pose, stop the robot. This is to avoid the robot drive in circles
        # continuously
        if distance_to_goal < 0.08:
            self.speed = 0

    def sendCommandCallback(self, event):
        """
        Callback function that send the twist command to the robot (speed and angle needed to go to the goal pose)
        :param event: not used.
        """
        print('Sending twist command')

        # Check if the goal pose is active or not
        if not self.goal_active:  # No goal, no movement
            self.speed = 0
            self.angle = 0
        else:
            self.driveStraight()

        # Construct the twist message for the robot with the speed and angle needed
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.angle

        # Publish the twist command to the robot
        self.publisher_command.publish(twist)


def publisher():
    # ----------------------------------
    # Initialization
    # ----------------------------------

    # Initialize node and call class instance
    rospy.init_node('p_ldalcol_driver', anonymous=False)  # Initialize the node
    driver = Driver()

    rate = rospy.Rate(10)  # time rate of the message

    rospy.spin()

    # # ----------------------------------
    # # Execution
    # # ----------------------------------
    # while not rospy.is_shutdown():
    #     twist = Twist()
    #     twist.linear.x = 0.1
    #     twist.angular.z = -1
    #
    #     publisher.publish(twist)
    #     rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
