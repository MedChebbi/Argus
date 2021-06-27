#!/usr/bin/env python3

import rospy
from argus_msgs.msg import Distance
from geometry_msgs.msg import Twist

def my_callback(msg):
    distance_moved_x = msg.distance_x
    distance_moved_y = msg.distance_y
    if distance_moved_x < 1.5:
        speed.linear.x = 0.1
        if distance_moved_y > 0.03:
            speed.angular.z = -0.2
        elif distance_moved_y < 0.03:
            speed.angular.z = 0.2
        else:
            speed.angular.z = 0.0
    else:
        speed.linear.x = 0
    pub.publish(speed)


rospy.init_node("test_movement_node")
rospy.Subscriber('control/moved_distance', Distance, my_callback)
pub = rospy.Publisher("control/cmd_vel", Twist, queue_size=1)
speed = Twist()
rospy.spin()
