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

from intersection_detection import LineStateClassifier
from argus_computer_vision.cfg import lineParamConfig
from dynamic_reconfigure.server import Server

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
        self.pub = rospy.Publisher("/detected_line/image", Image, queue_size=1)
        self.pub2 = rospy.Publisher("/detected_line/info", LineInfo, queue_size=1)
        self.width = width
        self.height = height
        self.debug = debug
        self.img = Image()
        self.warp_img = Image()
        self.msg = LineInfo()
        self.error = 0
        self.ang = 0
        self.line_param = {"Width_Top": 100, "Height_Top": 180, "Width_Bottom": 0,
		                  "Height_Bottom" : 360,"THRES": 25, "MIN_AREA": 15000, "MAX_AREA": 50000}
        srv = Server(lineParamConfig, self.reconfig)
        self.sub = rospy.Subscriber("camera/image_raw", Image, self.callback, queue_size = 1, buff_size=2**24)

    def publish(self):
        self.pub.publish(self.img)
        self.pub2.publish(self.msg)

    def shutdown(self):
        self.msg.detected = False
        self.pub2.publish(self.msg)
        rospy.loginfo("shuting down line detection!")


    def reconfig(self, config, level):
        self.line_param = config
        return config

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data,'bgr8')
            self.width = image.shape[1]
            self.height = image.shape[0]
            widthTop = self.line_param["Width_Top"]
            heightTop = self.line_param["Height_Top"]
            widthBottom = self.line_param["Width_Bottom"]
            heightBottom = self.line_param["Height_Bottom"]
            points = np.float32([(widthTop, heightTop), (self.width - widthTop, heightTop),
                                 (widthBottom , heightBottom ), (self.width - widthBottom, heightBottom)])
            img = drawPoints(image.copy(), points)
            imgWarped = self.warpImg (image, points)

            gray = cv2.cvtColor(imgWarped,cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray,(7,7),1)
            ret,thr = cv2.threshold(gray,20,255,cv2.THRESH_BINARY)
            if self.run_ml_model:
                pred_class, preds = self.classifier.predict(thr)
                self.msg.state = pred_class
            else:
                self.msg.state = "None"
            out_image, bwImg, info = self.detBlack(imgWarped, draw = self.debug)
            self.msg.detected = info[0]
            self.msg.error = info[1]
            self.msg.angle = info[2]
            if self.debug:
                rospy.loginfo(self.msg)
            self.img = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            self.publish()
            rospy.on_shutdown(self.shutdown)
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

    def detBlack(self, imgWarped, draw = True):
        black_detected = False
        coff = 10
        needed_info = []
        max_area_thre = self.line_param["MAX_AREA"]
        min_area_thre = self.line_param["MIN_AREA"]
        WIDTH = self.width
        HEIGHT = self.height
        x_last = WIDTH//2
        y_last = HEIGHT//2
        roi = imgWarped.copy()
        if draw:
            image = imgWarped.copy()
        mask = int(HEIGHT/2.5)
        roi[0:mask,:] = (255,255,255)      #(B, G, R)
        #roi[HEIGHT-mask:HEIGHT,:] = (255,255,255)
        roi = cv2.GaussianBlur(roi,(5,5),1)
        thr = self.line_param["THRES"]
        Blackline= cv2.inRange(roi, (0,0,0), (thr,thr,thr))

        kernel = np.ones ((3,3), np.uint8)
        Blackline = cv2.erode(Blackline, kernel, iterations=2)
        Blackline = cv2.dilate(Blackline, kernel, iterations=2)

        contours, hierarchy = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contours_len = len(contours)
        areas = [0]
        for i in contours:
            area = cv2.contourArea(i)
            areas.append(area)
        max_area = max(areas)
        #print (max_area)
        if contours_len > 0 and (min_area_thre < max_area < max_area_thre):
            black_detected = True

            if len(contours) == 1:
                x,y,w,h = cv2.boundingRect(contours[0])
                blackbox = cv2.minAreaRect(contours[0])

            else:
                dets = []
                off_bottom = 0
                for cont_num in range (contours_len):
                    blackbox = cv2.minAreaRect(contours[cont_num])
                    (x_min, y_min), (w_min, h_min), ang = blackbox
                    box = cv2.boxPoints(blackbox)
                    (x_box, y_box) = box[0]
                    if y_box > HEIGHT-mask-1 :
                        off_bottom += 1
                    dets.append((y_box,cont_num,x_min,y_min))
                dets = sorted(dets)
                if off_bottom > 1:
                    canditates_off_bottom=[]
                    for con_num in range ((contours_len - off_bottom), contours_len):
                        (y_highest,con_highest,x_min, y_min) = dets[con_num]
                        total_distance = (abs(x_min - x_last)**2 + abs(y_min - y_last)**2)**0.5
                        canditates_off_bottom.append((total_distance,con_highest))
                    canditates_off_bottom = sorted(canditates_off_bottom)
                    (total_distance,con_highest) = canditates_off_bottom[0]
                    blackbox = cv2.minAreaRect(contours[con_highest])
                else:
                    (y_highest,con_highest,x_min, y_min) = dets[contours_len-1]
                    blackbox = cv2.minAreaRect(contours[con_highest])

            setpoint = WIDTH//2
            (x_min, y_min), (w_min, h_min), ang = blackbox
            x_last = x_min
            y_last = y_min

            if (ang < -45) or (w_min > h_min and ang < 0): ang += 90
            if (w_min < h_min and ang > 0): ang = (90-ang)*-1

            ang = int(ang)
            box = cv2.boxPoints(blackbox)
            box = np.int0(box)
            error = int(x_min - setpoint)
            centertext = "Offset: " + str(error)
            if draw:
                cv2.drawContours(image,[box],0,(0,0,255),3)
                cv2.putText(image,"Angle: "+str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, self.RED, 2)
                cv2.drawContours(image, contours, -1, self.GREEN, 3)
                cv2.putText(image, centertext, (200,340), cv2.FONT_HERSHEY_SIMPLEX, 1, self.RED,2)
                cv2.circle(image, (WIDTH//2, HEIGHT//2),5, self.BLUE,cv2.FILLED)
                cv2.line(image, (int(x_min), 200), (int(x_min), 250),self.BLUE,3)
            needed_info.append(black_detected)
            self.error = error//coff
            self.ang = ang
            needed_info.append(self.error)
            needed_info.append(self.ang)
        else :
            black_detected = False
            needed_info.append(black_detected)
            needed_info.append(self.error)
            needed_info.append(self.ang)
        if draw: cv2.putText(image,"State: "+self.msg.state, (10,80),
                             cv2.FONT_HERSHEY_SIMPLEX, 1, self.RED,2)
        return image, Blackline, needed_info

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
    rospy.set_param('classifier/running', True)
    rospy.set_param('classifier/input_shape', [64,64,1])
    rospy.set_param('classifier/number_classes', 6)
    rospy.set_param('classifier/class_names', ['straight', 'x', 'T', 'left', 'right', 'end'])
    rospy.set_param('classifier/threshold', 0.65)
    rospy.set_param('classifier/queue_size', 3)
    rospy.set_param('classifier/model_path', '/home/mohamed/robolympix/argus_ws/src/argus_computer_vision/scripts/models/model_grayscale.tflite')
    rospy.set_param('classifier/on_edge', False)
    rospy.set_param('classifier/debug', True)

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
