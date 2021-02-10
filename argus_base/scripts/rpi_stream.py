#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from argus_base.cfg import imgParamConfig
from dynamic_reconfigure.server import Server
import numpy as np

class Streamer:
	def __init__(self):
		self.pub = rospy.Publisher("raw_image", Image, queue_size=1)
		self.width = 480
		self.height = 360
		self.brightness = 55
		self.fps = 30
		self.RED = (0,0,255)
		self.BLUE = (255,0,0)
		self.GREEN = (0,255,0)
		self.BLACK = (0,0,0)
		self.WHITE = (255,255,255)
		self.rate = rospy.Rate(20)
		self.msg = Image()
		self.param = {"LUM": 50,"size":1}
		srv = Server(imgParamConfig, self.reconfig)
		self.bridge = CvBridge()
		self.read()

	def reconfig(self, config, level):
		self.param = config
		print(self.param)
		return config

	def read(self):
		cap = cv2.VideoCapture(0,cv2.CAP_V4L)
		cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
		cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
		cap.set(10,self.brightness)
		self.width = cap.get(3)
		self.height = cap.get(4)
		print(self.width)
		print(self.height)
		while not rospy.is_shutdown():
			ret,frame = cap.read()
			timer = cv2.getTickCount()
			if ret:
				cap.set(10, self.param["LUM"])
				if self.param["size"] == 0:
					dim = (320,240)
				elif self.param["size"] == 1:
					dim = (480,360)
				elif self.param["size"] == 2:
					dim = (640,480)
				elif self.param["size"] == 3:
					dim = (1280,720)

				if cv2.waitKey(10) & 0xFF == ord('q'):
					break
				resized_frame = cv2.resize(frame,dim)
				self.width = resized_frame.shape[1]
				self.height = resized_frame.shape[0]
				print(self.width)
				print(self.height)
				frame2 = self.display_esti_dist(resized_frame)
				frame2 = self.drawFPS(frame2)
				cv2.imshow('output2',frame2)
				#cv2.imshow('output',frame)
				self.msg = self.bridge.cv2_to_imgmsg(resized_frame,'bgr8')
				self.publish()
				self.fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
		cap.release()
		cv2.destroyAllWindows()

	def display_esti_dist(self, frame):
		WIDTH = self.width
		HEIGHT = self.height
		frame2 = frame.copy()
		X1 = int(WIDTH//3)
		Y1 = int((HEIGHT*2)//3)
		X2 = int((WIDTH*2)//3)
		Y2 = int((HEIGHT*2)//3)
		cv2.line(frame2, (int(WIDTH//2),int(HEIGHT)),(int(WIDTH//2),Y2), self.GREEN,1)
		cv2.line(frame2, (X1,Y1),(X2,Y2), self.GREEN,1)
		cv2.putText(frame2,"10", (X1 - 5, Y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.GREEN, 1)
		return frame2

	def drawFPS(self, img):
		if self.fps>30: myColor = self.GREEN
		elif self.fps>4: myColor = self.BLUE
		else: myColor = self.RED
		cv2.putText(img, "Fps:", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.BLUE, 2);
		cv2.putText(img, str(int(self.fps)), (75, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, myColor, 2);
		return img

	def publish(self):
		self.pub.publish(self.msg)
		self.rate.sleep()

def main():
	rospy.init_node("rpicam_streaming_node", anonymous=True)
	try:
		stream = Streamer()
	except rospy.ROSInterruptException as e:
		print(e)

if __name__ == '__main__':
	main()
