#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from argus_msgs.msg import LineInfo
import sys, select, os
import time
import warnings
from argus_base.cfg import pidParamConfig
from dynamic_reconfigure.server import Server

_current_time = rospy.get_time

class PID(object):
    """
    A simple PID controller.
    """
    def __init__(self,
                 setpoint=0,
                 sample_time=0.01,
                 output_limits=(None, None),
                 auto_mode=True,
                 proportional_on_measurement=False):
        """
        :param setpoint: The initial setpoint that the PID will try to achieve
        :param sample_time:
        :param output_limits:
        :param auto_mode: Whether the controller should be enabled (in auto mode) or not (in manual mode)
        :param proportional_on_measurement: Whether the proportional term should be calculated on the input directly
                                            rather than on the error (which is the traditional way). Using
                                            proportional-on-measurement avoids overshoot for some types of systems.
        """
        self.setpoint = setpoint
        self.sample_time = sample_time

        self._min_output, self._max_output = output_limits
        self._auto_mode = auto_mode
        self.proportional_on_measurement = proportional_on_measurement

        self.pid_param = {"KP": 0.05, "KI": 0, "KD": 0}
        srv = Server(pidParamConfig, self.reconfig)

        self.Kp, self.Ki, self.Kd = self.pid_param["KP"], self.pid_param["KI"], self.pid_param["KD"]

        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._last_time = _current_time()
        self._last_output = None
        self._last_input = None

    def __call__(self, input_):
        if not self.auto_mode:
            return self._last_output

        now = _current_time()
        dt = now - self._last_time if now - self._last_time else 1e-16

        if self.sample_time is not None and dt < self.sample_time and self._last_output is not None:
            # only update every sample_time seconds
            return self._last_output

        # compute error terms
        error = self.setpoint - input_
        d_input = input_ - (self._last_input if self._last_input is not None else input_)

        # compute the proportional term
        if not self.proportional_on_measurement:
            # regular proportional-on-error, simply set the proportional term
            self._proportional = self.Kp * error
        else:
            # add the proportional error on measurement to error_sum
            self._proportional -= self.Kp * d_input
            self._proportional = _clamp(self._proportional, self.output_limits)

        # compute integral and derivative terms
        self._integral += self.Ki * error * dt
        self._integral = _clamp(self._integral, self.output_limits)  # avoid integral windup

        self._derivative = -self.Kd * d_input / dt

        # compute final output
        output = self._proportional + self._integral + self._derivative
        output = _clamp(output, self.output_limits)

        # keep track of state
        self._last_output = output
        self._last_input = input_
        self._last_time = now

        return output

    def reconfig(self, config, level):
        self.pid_param = config
        print(self.pid_param)
        return config

    @property
    def components(self):
        """
        The P-, I- and D-terms from the last computation as separate components as a tuple. Useful for visualizing
        what the controller is doing or when tuning hard-to-tune systems.
        """
        return self._proportional, self._integral, self._derivative

    @property
    def tunings(self):
        """The tunings used by the controller as a tuple: (Kp, Ki, Kd)"""
        return self.Kp, self.Ki, self.Kd

    @tunings.setter
    def tunings(self, tunings):
        """Setter for the PID tunings"""
        self.Kp, self.Ki, self.Kd = tunings

    @property
    def auto_mode(self):
        """Whether the controller is currently enabled (in auto mode) or not"""
        return self._auto_mode

    @auto_mode.setter
    def auto_mode(self, enabled):
        """Enable or disable the PID controller"""
        if enabled and not self._auto_mode:
            # switching from manual mode to auto, reset
            self._last_output = None
            self._last_input = None
            self._last_time = _current_time()
            self._proportional = 0
            self._integral = _clamp(0, self.output_limits)

        self._auto_mode = enabled

    def set_auto_mode(self, enabled, last_output=None):

        if enabled and not self._auto_mode:
            # switching from manual mode to auto, reset
            self._last_output = last_output
            self._last_input = None
            self._last_time = _current_time()
            self._proportional = 0
            self._integral = (last_output if last_output is not None else 0)
            self._integral = _clamp(self._integral, self.output_limits)

        self._auto_mode = enabled

    @property
    def output_limits(self):
        """
        The current output limits as a 2-tuple: (lower, upper). See also the *output_limts* parameter in
        :meth:`PID.__init__`.
        """
        return self._min_output, self._max_output

    @output_limits.setter
    def output_limits(self, limits):
        """Setter for the output limits"""
        if limits is None:
            self._min_output, self._max_output = None, None
            return

        min_output, max_output = limits

        if None not in limits and max_output < min_output:
            raise ValueError('lower limit must be less than upper limit')

        self._min_output = min_output
        self._max_output = max_output

        self._integral = _clamp(self._integral, self.output_limits)
        self._last_output = _clamp(self._last_output, self.output_limits)

def _clamp(value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif upper is not None and value > upper:
        return upper
    elif lower is not None and value < lower:
        return lower
    return value

class Controller(object):
	def __init__(self):
		rospy.init_node("argus_head_node")
		self.pub = rospy.Publisher("control/cmd_vel", Twist, queue_size=1)
		self.rate = rospy.Rate(10)
		self.vel = Twist()
		self.line_info = LineInfo()
		self.auto_mode = Bool()
		self.auto_mode = True
		self.pid = PID(setpoint=0,output_limits=(-1.0, 1.0))
		
		self.line_follow_sub = rospy.Subscriber("detected_line_info", LineInfo , self.lineCallback)
		self.vel_sub = rospy.Subscriber('cmd_vel', Twist, self.callback)
		self.teleop_sub = rospy.Subscriber('teleop/command', Bool, self.teleCallback)

	def publish(self):
		self.pub.publish(self.vel)
		self.rate.sleep()

	def callback(self, data):
		if self.auto_mode == False:
			self.vel.linear.x = data.linear.x
			self.vel.linear.y = data.linear.y
			self.vel.linear.z = data.linear.z
			self.vel.angular.x= data.angular.x
			self.vel.angular.y = data.angular.y
			self.vel.angular.z = data.angular.z
			print("robot speed: \n", str(self.vel))
			self.publish()
		else:
			print("auto mode initiated")
			pass
		
	def lineCallback(self, data):
		self.line_info = data
		if self.line_info.detected and self.auto_mode == True:
			self.vel.linear.x = 0.3
			self.vel.linear.y = 0
			self.vel.linear.z = 0
			self.vel.angular.x= 0
			self.vel.angular.y = 0
			self.vel.angular.z = self.pid(self.line_info.error)
			print("after pid robot speed: \n", str(self.vel))
		elif (not self.line_info.detected) and self.auto_mode == True:
			self.vel.linear.x = -0.3
			self.vel.linear.y = 0
			self.vel.linear.z = 0
			self.vel.angular.x= 0
			self.vel.angular.y = 0
			self.vel.angular.z = 0
			print("no line in auto mode \n robot speed: \n", str(self.vel))
		self.publish()
		#print(self.line_info.error)
		
	def teleCallback(self, data):
		self.auto_mode = data
		print(self.auto_mode)

if __name__=="__main__":
	controller = Controller()
	try:
		rospy.spin()
	except rospy.ROSInterruptException as e:
		print(e)
