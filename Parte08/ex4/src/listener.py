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

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import argparse
import rospy
from std_msgs.msg import String
from ex4.msg import Dog


def callbackMessageReceived(msg):
    """
    Function that is called when the message arrives at the subscriber
    :param msg:
    """
    rospy.loginfo('I received a dog named ' + msg.name + ', which is ' + str(msg.age) + ' years old and is '
                  + msg.color + '. His brothers are: ' + str(msg.brothers))  # Print the message


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    # ----------------------------------
    # Initialization
    # ----------------------------------
    ap = argparse.ArgumentParser()
    ap.add_argument('-t', '--topic', type=str, action='append', required=True,
                    help="Define the name of the topic that you want to subscribe")
    args = vars(ap.parse_args())

    rospy.init_node('listener', anonymous=True)  # Initialize the node
    for topic in args['topic']:
        rospy.Subscriber(topic, Dog, callbackMessageReceived)  # Subscribe the node to the specified topic

    # ----------------------------------
    # Execution
    # ----------------------------------
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
