#!/usr/bin/env python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import argparse

import colorama
import rospy
from std_msgs.msg import String
from psr_parte09_exs.msg import Dog
from geometry_msgs.msg import Twist


def publisher():
    # ----------------------------------
    # Initialization
    # ----------------------------------
    rospy.init_node('p_ldalcol_driver', anonymous=False)  # Initialize the node
    publisher = rospy.Publisher('p_ldalcol/cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(10)  # time rate of the message

    # ----------------------------------
    # Execution
    # ----------------------------------
    while not rospy.is_shutdown():

        # Read global parameter
        highlight_text_color = rospy.get_param('/highlight_text_color')

        # Sent dog message
        dog = Dog()
        dog.name = 'max'
        dog.age = 7
        dog.color = 'black'
        dog.brothers.append('Lily')
        dog.brothers.append('Boby')
        rospy.loginfo('Here it goes a dog named ' + getattr(colorama.Fore, highlight_text_color) + dog.name +
                      colorama.Style.RESET_ALL + '.')
        publisher.publish(dog)

        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
