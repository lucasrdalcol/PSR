#!/usr/bin/env python3

# # --------------------------------------------------
# # EXERCISE 5 - PSR
# # --------------------------------------------------
#
# from __future__ import print_function
# import rospy
# from ex5.srv import *
#
#
# def set_dog_name_client(new_name):
#     rospy.wait_for_service('set_dog_name')
#     try:
#         set_dog_name = rospy.ServiceProxy('set_dog_name', SetDogName)
#         resp1 = set_dog_name(new_name)
#         return resp1.result
#     except rospy.ServiceException as e:
#         print("Service call failed: %s" % e)
#
#
# def usage():
#     return "%s new dogs name" % sys.argv[0]
#
#
# if __name__ == "__main__":
#     new_name = 'Lucas'
#     print("Requesting new dog's name: " + new_name)
#     print("New dog's name: " + set_dog_name_client(new_name))

# --------------------------------------------------
# EXAMPLE ROS TUTORIALS
# --------------------------------------------------

from __future__ import print_function
import sys
import rospy
from beginner_tutorials.srv import *


def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage():
    return "%s [x y]" % sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s" % (x, y))
    print("%s + %s = %s" % (x, y, add_two_ints_client(x, y)))
