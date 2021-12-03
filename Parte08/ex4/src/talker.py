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

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import argparse
import rospy
from std_msgs.msg import String
from ex4.msg import Dog


def talker():
    # ----------------------------------
    # Initialization
    # ----------------------------------
    ap = argparse.ArgumentParser()
    ap.add_argument('-r', '--rate', type=float, default=10, help="Define time rate in Hertz")
    ap.add_argument('-t', '--topic', type=str, required=True,
                    help="Define the name of the topic that you want to publish")
    # ap.add_argument('-m', '--message', type=str, default='I do not know what to say', help="Define the message to be "
    #                                                                                        "sent")
    args = vars(ap.parse_args())

    rospy.init_node('talker', anonymous=True)  # Initialize the node
    pub = rospy.Publisher(args['topic'], Dog, queue_size=10)
    # pub2 = rospy.Publisher('A17', String, queue_size=10)
    rate = rospy.Rate(args['rate'])  # time rate of the message

    # ----------------------------------
    # Execution
    # ----------------------------------
    while not rospy.is_shutdown():
        # # Sent message from argparse
        # message = args['message']  # Message to be sent
        # rospy.loginfo(message)  # print the message in the terminal of the talker
        # pub2.publish(message)  # publish the message in the topic

        # Sent dog message
        dog = Dog()
        dog.name = 'max'
        dog.age = 7
        dog.color = 'black'
        dog.brothers.append('Lily')
        dog.brothers.append('Boby')
        rospy.loginfo('Here it goes my dog.')
        pub.publish(dog)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
