#!/usr/bin/env python3

from controller import PID
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from argus_msgs.msg import LineInfo
from argus_base.cfg import pidParamConfig
from dynamic_reconfigure.server import Server

class Controller(object):
	def __init__(self):
		self.pub = rospy.Publisher("control/cmd_vel", Twist, queue_size=1)
		self.rate = rospy.Rate(10)
		self.vel = Twist()
		self.vel.linear.x = 0
		self.vel.linear.y = 0
		self.vel.linear.z = 0
		self.vel.angular.x= 0
		self.vel.angular.y = 0
		self.vel.angular.z = 0
		self.line_info = LineInfo()
		self.auto_mode = Bool()
		self.auto_mode = True
		self.pid_param = {"KP": 0.25, "KI": 0, "KD": 0}
		srv = Server(pidParamConfig, self.reconfig)
		self.Kp, self.Ki, self.Kd = self.pid_param["KP"], self.pid_param["KI"], self.pid_param["KD"]
		self.pid = PID(self.Kp, self.Ki, self.Kd,setpoint=0,output_limits=(-5.0, 5.0))
		self.publish()
		self.teleop_sub = rospy.Subscriber('/teleop/command', Bool, self.teleCallback)
		self.line_follow_sub = rospy.Subscriber("detected_line_info", LineInfo , self.lineCallback)
		self.vel_sub = rospy.Subscriber('/teleop/cmd_vel', Twist, self.callback)
	
	def run(self):
		while not rospy.is_shutdown():
			if self.auto_mode == True:
				print("automatic")
				print(self.line_info.error)
				if self.line_info.detected == True:
					self.vel.linear.x = 0.4
					self.vel.angular.z = self.pid(self.line_info.error)
					print("after pid robot speed: \n", str(self.vel))
				else:
					self.vel.linear.x = -0.4
					print("no line in auto mode \n robot speed: \n", str(self.vel))
				self.publish()
			else:
				print("manual")
				print("robot speed: \n", str(self.vel))
				self.publish()

	def publish(self):
		self.pub.publish(self.vel)
		self.rate.sleep()
		print("publishing")

	def callback(self, data):
		self.vel = data

	def lineCallback(self, data):
		self.line_info = data

	def teleCallback(self, data):
		print(data)
		self.auto_mode = data.data
		print(self.auto_mode)
	def reconfig(self, config, level):
		self.pid_param = config
		print(self.pid_param)
		return config

if __name__=="__main__":
	rospy.init_node("argus_head_node")
	controller = Controller()
	try:
		controller.run()
	except rospy.ROSInterruptException as e:
		controller.vel.linear.x = 0
		controller.vel.angular.z = 0
		controller.publish()
		print(e)
