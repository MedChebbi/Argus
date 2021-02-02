#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, os

def PID_control():
	pass

def pub_cmd_vel():
	pub=rospy.Publisher("control/cmd_vel" , Twist , queue_size=1)
	return pub

def process_data(data, publisher):
	vel= Twist()
	vel.linear.x = data.linear.x
	vel.linear.y = data.linear.y
	vel.linear.z = data.linear.z
	vel.angular.x= data.angular.x
	vel.angular.y = data.angular.y
	vel.angular.z = data.angular.z
	print("robot speed: \n", str(vel))


	publisher.publish(vel)



def sub_cmd_vel(pub):
	rospy.Subscriber("cmd_vel", Twist , process_data,(pub))

if __name__=="__main__":

	try:
		rospy.init_node("argus_head_node")
		while not rospy.is_shutdown():
			rate = rospy.Rate(10)
			pub=pub_cmd_vel()
			sub_cmd_vel(pub)
			rate.sleep()

	except rospy.ROSInterruptException:
		print('rosException')
		pass
