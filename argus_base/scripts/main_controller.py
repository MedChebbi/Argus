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
		self.auto_mode = False
		self.pid_param = {"KP": 0.05, "KI": 0, "KD": 0}
		srv = Server(pidParamConfig, self.reconfig)
		self.Kp, self.Ki, self.Kd = self.pid_param["KP"], self.pid_param["KI"], self.pid_param["KD"]
		self.pid = PID(self.Kp, self.Ki, self.Kd,setpoint=0,output_limits=(-1.0, 1.0))
		self.publish()
		self.teleop_sub = rospy.Subscriber('/teleop/command', Bool, self.teleCallback)
		self.line_follow_sub = rospy.Subscriber("detected_line_info", LineInfo , self.lineCallback)
		self.vel_sub = rospy.Subscriber('/teleop/cmd_vel', Twist, self.callback)
		
	def publish(self):
		self.pub.publish(self.vel)
		print("publishing")

	def callback(self, data):
		if self.auto_mode == False:
			self.vel = data
			print("robot speed: \n", str(self.vel))
		else:
			pass
		self.publish()

	def lineCallback(self, data):
		self.line_info = data
		if self.auto_mode == True:

			print(self.line_info.error)
			if self.line_info.detected == True:
				self.vel.linear.x = 0.4
				self.vel.angular.z = self.pid(self.line_info.error)
				print("after pid robot speed: \n", str(self.vel))

			else:
				self.vel.linear.x = -0.4
				print("no line in auto mode \n robot speed: \n", str(self.vel))
		else:
			pass
		self.publish()

	def teleCallback(self, data):
		self.auto_mode = data
		#print(self.auto_mode)

	def reconfig(self, config, level):
		self.pid_param = config
		print(self.pid_param)
		return config

if __name__=="__main__":
	rospy.init_node("argus_head_node")
	controller = Controller()
	try:
		rospy.spin()
		print("done")
	except rospy.ROSInterruptException as e:
		print(e)
