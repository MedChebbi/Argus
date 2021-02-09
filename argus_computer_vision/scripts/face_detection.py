#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class FaceDetect():
	"""docstring for ."""
	def __init__(self):
		rospy.init_node("face_detection")
        self.face_cascade = cv2.CascadeClassifier('models/cascades/haarcascade_frontalface_alt2.xml')
		self.bridge = CvBridge()
		self.pub = rospy.Publisher("detected_face_image", Image, queue_size=1)
		#self.pub2 = rospy.Publisher("detected_line_info", LineInfo, queue_size=1)
		self.width = 480
		self.height = 360
		self.rate = rospy.Rate(10)
		self.img = Image()
		#self.msg = FaceInfo()
		self.RED = (0,0,255)
		self.BLUE = (255,0,0)
		self.GREEN = (0,255,0)
		self.BLACK = (0,0,0)
		self.WHITE = (255,255,255)

		self.sub = rospy.Subscriber("raw_image", Image, self.callback, queue_size = 1, buff_size=2**24)

	def publish(self):
		self.pub.publish(self.img)
		#self.pub2.publish(self.msg)
		self.rate.sleep()

    def detect_face(self,img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=2)
        for (x, y, w, h) in faces:
            width = x+w
            height = y+h
            cv2.rectangle(img,(x,y),(width,height),self.BLUE,1)
        cv2.imshow('face detected',img)
        return img

	def callback(self, data):
		try:
			image = self.bridge.imgmsg_to_cv2(data,'bgr8')
			self.width = image.shape[1]
			self.height = image.shape[0]

            img = detect_face(image.copy())
			self.img = self.bridge.cv2_to_imgmsg(img, "bgr8")
			self.publish()
		except CvBridgeError as e:
			print(e)

if __name__ == '__main__':
    face_detector = FaceDetect()
	try:
		rospy.spin()
	except rospy.ROSInterruptException as e:
		print(e)
	finally:
		cv2.destroyAllWindows()
