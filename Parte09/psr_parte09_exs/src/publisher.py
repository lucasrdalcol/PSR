#!/usr/bin/env python3

# --------------------------------------------------
# LICENSE
# --------------------------------------------------
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple publisher demo that published std_msgs/Strings messages
## to the 'chatter' topic

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import argparse

import colorama
import rospy
from std_msgs.msg import String
from psr_parte09_exs.msg import Dog


def publisher():
    # ----------------------------------
    # Initialization
    # ----------------------------------
    rospy.init_node('publisher', anonymous=True)  # Initialize the node
    pub = rospy.Publisher('chatter', Dog, queue_size=10)

    # Read private parameter
    frequency = rospy.get_param('~frequency', default=1)

    rate = rospy.Rate(frequency)  # time rate of the message

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
        pub.publish(dog)

        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
