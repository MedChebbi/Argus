#!/usr/bin/env python3

import rospy
import cv2
try:
    from cv_bridge import CvBridge, CvBridgeError
except:
    pass
import numpy as np

from sensor_msgs.msg import Image
from argus_msgs.msg import LineInfo

from argus_computer_vision.cfg import lineParamConfig
from dynamic_reconfigure.server import Server

from intersection_detection import LineStateClassifier
from color_detection import ColorDetector


class LineDetector:
    """docstring for ."""
    RED = (0,0,255)
    BLUE = (255,0,0)
    GREEN = (0,255,0)
    BLACK = (0,0,0)
    WHITE = (255,255,255)

    def __init__(self,width = 480 ,height = 360, debug = True):
        self.bridge = CvBridge()
        
        #self.c = 0
        classifier_params = rospy.get_param('/classifier')
        self.run_ml_model = classifier_params['running']
        if self.run_ml_model: self.classifier = LineStateClassifier(classifier_params)
        self.pub_img = rospy.Publisher("/detected_line/image", Image, queue_size=1)
        self.pub_debug = rospy.Publisher("/detected_blob/image", Image, queue_size=1)
        self.pub_info = rospy.Publisher("/detected_line/info", LineInfo, queue_size=1)
        self.width = width
        self.height = height
        self.debug = debug
        self.img = Image()
        self.debug_img = Image()
        self.warp_img = Image()
        self.msg = LineInfo()
        self.error = 0
        self.ang = 0
        self.line_param = {"Width_Top": 100, "Height_Top": 180, "Width_Bottom": 0,
		                  "Height_Bottom" : 360,"THRES": 25, "MIN_AREA": 15000, "MAX_AREA": 50000}
        

        srv = Server(lineParamConfig, self.reconfig)

        black_line_params = rospy.get_param('/black_line')
        blob_params = rospy.get_param('/color_blob')

        self.black_line_detector = ColorDetector(black_line_params)
        self.color_detector = ColorDetector(blob_params)

        self.sub = rospy.Subscriber("camera/image_raw", Image, self.callback, queue_size = 1, buff_size=2**24)


    def publish(self):
        self.pub_info.publish(self.msg)
        self.pub_debug.publish(self.debug_img)
        self.pub_img.publish(self.img)


    def shutdown(self):
        self.msg.detected = False
        self.pub_info.publish(self.msg)
        rospy.loginfo("shuting down line detection!")


    def reconfig(self, config, level):
        self.line_param = config
        return config


    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data,'bgr8')
            self.width = image.shape[1]
            self.height = image.shape[0]
            
            # Image warping
            widthTop = self.line_param["Width_Top"]
            heightTop = self.line_param["Height_Top"]
            widthBottom = self.line_param["Width_Bottom"]
            heightBottom = self.line_param["Height_Bottom"]

            points = np.float32([(widthTop, heightTop), (self.width - widthTop, heightTop),
                                 (widthBottom , heightBottom ), (self.width - widthBottom, heightBottom)])
            img = drawPoints(image.copy(), points)
            imgWarped = self.warpImg (image, points)
            
            # Running line state detection 
            if self.run_ml_model:
                gray = cv2.cvtColor(imgWarped,cv2.COLOR_BGR2GRAY)
                gray = cv2.GaussianBlur(gray,(7,7),1)
                ret,thr = cv2.threshold(gray,20,255,cv2.THRESH_BINARY)
                pred_class, preds = self.classifier.predict(thr)
                self.msg.state = pred_class
            else:
                self.msg.state = "None"
        
            # Running color blob detection
            debug_frame, mask, color_info = self.color_detector.detect(image, image.copy(), draw = self.debug)

            # Black line detection
            out_image, bwImg, info = self.black_line_detector.detect(imgWarped, imgWarped.copy(), self.line_param, draw = self.debug)
            self.msg.detected = info[0]
            self.msg.error = info[1]
            self.msg.angle = info[2]
            if self.debug:
                cv2.putText(out_image,"State: "+self.msg.state, (10,80), cv2.FONT_HERSHEY_SIMPLEX, 1, self.RED,2)
                rospy.loginfo(self.msg)
                #rospy.loginfo(self.msg.error)
                #rospy.loginfo(self.msg.angle)

            self.img = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            self.debug_img = self.bridge.cv2_to_imgmsg(debug_frame, "bgr8")
            self.publish()
            rospy.on_shutdown(self.shutdown)
            # cv2.imshow('mask',mask)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     cv2.destroyAllWindows()
            '''
            cv2.imshow('warpedImg',thr)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()

            if cv2.waitKey(1) & 0xFF == ord('y'):
                self.c+=1
                cv2.imwrite('/home/mohamed/test/shot3_'+str(self.c)+'.jpg',thr)
                print('shot taken')

            #cv2.imshow('bw',bwImg)
            '''
        except CvBridgeError as e:
            rospy.logwarn(e)


    def warpImg (self,img,points):
        # print(points)
        points =reorder(points)
        pts1 = np.float32(points)
        pts2 = np.float32([[0,0],[self.width,0],[0,self.height],[self.width,self.height]])
        matrix = cv2.getPerspectiveTransform(pts1,pts2)
        imgWarp = cv2.warpPerspective(img,matrix,(self.width,self.height))
        return imgWarp


def drawPoints(img,points):
    for x in range(4):
        cv2.circle(img,(int(points[x][0]),int(points[x][1])),5,(0,0,255),cv2.FILLED)
    return img


def reorder(myPoints):
	myPoints = myPoints.reshape((4, 2))
	myPointsNew = np.zeros((4, 1, 2), dtype=np.int32)
	add = myPoints.sum(1)
	myPointsNew[0] = myPoints[np.argmin(add)]
	myPointsNew[3] =myPoints[np.argmax(add)]
	diff = np.diff(myPoints, axis=1)
	myPointsNew[1] =myPoints[np.argmin(diff)]
	myPointsNew[2] = myPoints[np.argmax(diff)]
	return myPointsNew


def set_default_params():
    #Set classifier parameters
    rospy.set_param('classifier/running', True)
    rospy.set_param('classifier/input_shape', [64,64,1])
    rospy.set_param('classifier/number_classes', 6)
    rospy.set_param('classifier/class_names', ['straight', 'x', 'T', 'left', 'right', 'end'])
    rospy.set_param('classifier/threshold', 0.65)
    rospy.set_param('classifier/queue_size', 3)
    rospy.set_param('classifier/model_path', '/home/mohamed/robolympics_ws/src/argus_computer_vision/scripts/models/model_grayscale.tflite')
    rospy.set_param('classifier/on_edge', True)
    rospy.set_param('classifier/debug', False)

    #Set color black line detector parameters
    rospy.set_param('black_line/color_space', 'BGR')
    rospy.set_param('black_line/color_min_range', [0, 0, 0])
    rospy.set_param('black_line/color_max_range', [25, 25, 25])
    rospy.set_param('black_line/MIN_AREA', 200)
    rospy.set_param('black_line/MAX_AREA', 80000)
    rospy.set_param('black_line/fine_tuning', False)
    rospy.set_param('black_line/detection_mode', "line")

    #Set color blob detector parameters
     #Red approx range in hsv: (0, 120, 30) ~ (25, 255, 255)
    rospy.set_param('color_blob/color_space', "HSV")
    rospy.set_param('color_blob/color_min_range', [0, 120, 20])
    rospy.set_param('color_blob/color_max_range', [25, 255, 255])
    rospy.set_param('color_blob/MIN_AREA', 3000)
    rospy.set_param('color_blob/MAX_AREA', 80000)
    rospy.set_param('color_blob/fine_tuning', False)
    rospy.set_param('color_blob/detection_mode', "blob")


if __name__ == '__main__':
    rospy.init_node("line_detection")
    set_default_params()
    blackLineDetector = LineDetector()
    try:
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logwarn(e)
    finally:
        cv2.destroyAllWindows()
