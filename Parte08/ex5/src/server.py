#!/usr/bin/env python3

# # --------------------------------------------------
# # EXERCISE 5 - PSR
# # --------------------------------------------------
#
# from __future__ import print_function
# from ex5.srv import *
# import rospy
#
#
# def handle_set_dog_name(req):
#     print('Returning new dogs name: ' + req.result)
#     return SetDogNameResponse(req.new_name)
#
#
# def set_dog_name_server():
#     rospy.init_node('set_dog_name_server')
#     s = rospy.Service('set_dog_name', SetDogName, handle_set_dog_name)
#     print("Ready to set new dog name")
#     rospy.spin()
#
#
# if __name__ == "__main__":
#     set_dog_name_server()

# --------------------------------------------------
# EXAMPLE ROS TUTORIALS
# --------------------------------------------------

from __future__ import print_function
from beginner_tutorials.srv import AddTwoInts, AddTwoIntsResponse
import rospy


def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]" % (req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)


def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()


if __name__ == "__main__":
    add_two_ints_server()
