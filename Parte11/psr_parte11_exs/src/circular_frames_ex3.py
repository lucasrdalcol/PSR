#!/usr/bin/env python3

import math
import rospy
# Because of transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import turtlesim.msg


def main():
    rospy.init_node('circular_frame')
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    rate = rospy.Rate(100)

    alpha = 0
    rho = rospy.get_param('~distance_to_parent')
    period = rospy.get_param('~period')

    while not rospy.is_shutdown():
        alpha += (1/period)/100
        if alpha > 2 * math.pi:
            alpha = 0

        t.header.stamp = rospy.Time.now()

        # Sun to mercury
        t.header.frame_id = rospy.remap_name('parent')
        t.child_frame_id = rospy.remap_name('child')
        t.transform.translation.x = rho * math.cos(alpha)
        t.transform.translation.y = rho * math.sin(alpha)
        t.transform.rotation.w = 1

        # Sent transformation
        br.sendTransform(t)

        # Sleep
        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
