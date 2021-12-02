#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from argus_msgs.msg import Distance


class MovementDetector(object):
    def __init__(self):
        self.moved_dist = Distance()
        self.moved_dist.distance_x = 0.0
        self.moved_dist.distance_y = 0.0
        self.moved_dist.distance = 0.0
        self.get_init_position()

        self.distance_moved_pub = rospy.Publisher("control/moved_distance", Distance, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def get_init_position(self):
        data_odom = None
        while data_odom is None:
            try:
                data_odom = rospy.wait_for_message('/odom', Odometry, timeout = 1)
            except:
                rospy.loginfo("Current odom not ready yet, retrying for setting up init position")
        self._current_position = Point()
        self._current_position.x = data_odom.pose.pose.position.x
        self._current_position.y = data_odom.pose.pose.position.y
        self._current_position.z = data_odom.pose.pose.position.z

    def odom_callback(self, msg):
        NewPosition = msg.pose.pose.position
        
        if NewPosition.x >= self._current_position.x:
            self.moved_dist.distance_x += self.calculate_distance_x(NewPosition, self._current_position)
        else:
            self.moved_dist.distance_x -= self.calculate_distance_x(NewPosition, self._current_position)

        if NewPosition.y >= self._current_position.y:
            self.moved_dist.distance_y += self.calculate_distance_y(NewPosition, self._current_position)
        else:
            self.moved_dist.distance_y -= self.calculate_distance_y(NewPosition, self._current_position)

        self.moved_dist.distance += self.calculate_distance(NewPosition, self._current_position)
        self.update_current_position(NewPosition)
        aux = Distance()
        if self.moved_dist.distance < 0.001 :
            aux.distance = 0.0
            self.distance_moved_pub.publish(aux)
        elif -0.001 < self.moved_dist.distance_x < 0.001:
            aux.distance_x = 0.0
            self.distance_moved_pub.publish(aux)
        elif -0.001 < self.moved_dist.distance_y < 0.001:
            aux.distance_y = 0.0
            self.distance_moved_pub.publish(aux)
        else:
            self.distance_moved_pub.publish(self.moved_dist)

    
    def update_current_position(self, new_position):
        self._current_position.x = new_position.x
        self._current_position.y = new_position.y
        self._current_position.z = new_position.z

    def calculate_distance(self, new_position, old_position):
        return math.hypot(new_position.x - old_position.x, new_position.y - old_position.y)

    def calculate_distance_x(self, new_position, old_position):
        return math.sqrt((new_position.x - old_position.x)**2)

    def calculate_distance_y(self, new_position, old_position):
        return math.sqrt((new_position.y - old_position.y)**2)

    def publish_moved_distance(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node("movement_detector_node", anonymous = True)
    movement_detector = MovementDetector()
    movement_detector.publish_moved_distance()
