#!/usr/bin/env python3

import rospy
import tensorflow as tf
import cv2
try:
    from cv_bridge import CvBridge, CvBridgeError
except:
    pass
import numpy as np

from sensor_msgs.msg import Image
from argus_msgs.msg import LineInfo
from std_msgs.msg import String

class LineStateClassifier:
    """docstring for ."""
    RED = (0,0,255)
    BLUE = (255,0,0)
    GREEN = (0,255,0)
    BLACK = (0,0,0)
    WHITE = (255,255,255)

    def __init__(self, shape = (64,64,1), number_classes, model_path, debug = True):
        self.num_classes = 6
        self.class_names = ['straight', 'x', 'T', 'left', 'right', 'end']
        self.shape = shape
        self.interpreter = tf.lite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.pub = rospy.Publisher("detected_line_image", Image, queue_size=1)
        self.pubImg = rospy.Publisher("detected_line_image", Image, queue_size=1)
        self.sub = rospy.Subscriber("camera/image_raw", Image, self.callback, queue_size = 1, buff_size=2**24)
        self.msg =
    def __predict(self,image):
         # Get input and output tensors.
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()

        # Test the model on random input data.
        input_shape = input_details[0]['shape']
        #set random np array to test
        #input_data = np.array(np.random.random_sample(input_shape), dtype=np.float32)

        img = cv2.imread('straight1.jpg')

        test_img = preprocess(img)
        pyplot.imshow(img)

        input_data = np.array(test_img, dtype=np.float32)
        interpreter.set_tensor(input_details[0]['index'], input_data)

        interpreter.invoke()

        # The function `get_tensor()` returns a copy of the tensor data.
        # Use `tensor()` in order to get a pointer to the tensor.
        output_data = interpreter.get_tensor(output_details[0]['index'])
        print(output_data)
        pred_index = np.argmax(output_data[0])
        print('The predicted class is:', class_names[pred_index] )

    def publish(self):
        self.pub.publish(self.img)
        self.pub2.publish(self.msg)
        self.pub3.publish(self.warp_img)

    def callback(self, data):


if __name__ == '__main__':
    rospy.init_node("line_features")
	lineState = LineStateClassifier()
	try:
		rospy.spin()
	except rospy.ROSInterruptException as e:
		print(e)
	finally:
		cv2.destroyAllWindows()
