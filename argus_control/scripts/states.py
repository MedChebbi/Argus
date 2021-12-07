#!/usr/bin/env python3

from controller import PID

from smach import State, StateMachine, Concurrence
from smach_ros import SimpleActionState, MonitorState, IntrospectionServer
import threading

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from argus_msgs.msg import LineInfo
from argus_control.cfg import pidParamConfig
from dynamic_reconfigure.server import Server

from time import sleep


X1, Y1 = [0.45,0.6], [-0.08,0.08]
X2, Y2 = [1.2,1.3], [-0.08,0.08]
X3, Y3 = [1.41,1.48],[-0.24,-0.16]
X4, Y4 = [1.41,1.48],[-0.6,-0.4]
X5, Y5 = [0.9,1.02],[-0.68,-0.62]


line_follow_outcomes = ['reached_p1', 'reached_p2', 'reached_p3', 'reached_p4',
                        'reached_p5', 'preempted']


class LineFollow(MonitorState):
    def __init__(self, topic, msg_type, max_checks = -1):
        # Your state initialization goes here
        State.__init__(self, outcomes=line_follow_outcomes)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel_pub = rospy.Publisher("control/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        self.vel = Twist()
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x= 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0
        
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.publish()

        self.line_info = LineInfo()

        self._topic = topic
        self._msg_type = msg_type
        self._cond_cb = self.execute_cb
        self._max_checks = max_checks
        self._n_checks = 0
        
        self._trigger_event = threading.Event()

        self.pid_param = {"KP": 0.1, "KI": 0, "KD": 0.01, "S": 0.1}

        srv = Server(pidParamConfig, self.reconfig)
        self.Kp, self.Ki, self.Kd = self.pid_param["KP"], self.pid_param["KI"], self.pid_param["KD"]

        self.pid = PID(self.Kp, self.Ki, self.Kd,sample_time=0.01,setpoint=0,output_limits=(-1.5, 1.5))


    def publish(self):
        self.cmd_vel_pub.publish(self.vel)
        print("publishing")


    def reconfig(self, config, level):
        self.pid_param = config
        return config


    def get_init_position(self):
        data_odom = None
        while data_odom is None:
            try:
                data_odom = rospy.wait_for_message('/odom', Odometry, timeout = 1)
            except:
                rospy.loginfo("Current odom not ready yet, retrying for setting up init position")
        self.position = Point()
        self.position.x = data_odom.pose.pose.position.x
        self.position.y = data_odom.pose.pose.position.y
        self.position.z = data_odom.pose.pose.position.z


    def odom_callback(self, msg):
        self.position = msg.pose.pose.position


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.cmd_vel_pub.publish(self.vel)
        rospy.sleep(1)

    def execute_cb(self, user_data, msg):
        # Your state execution goes here
        if self.preempt_requested():
            self.service_preempt()
            self.vel.linear.x = 0
            self.vel.angular.z = 0
            self.cmd_vel_pub.publish(self.vel)
            return 'preempted'
        self.line_info = msg
        
        #print("line info: ",self.line_info)
        print('POS: ', self.position)
            
        if X1[0]< self.position.x < X1[1] and Y1[0] < self.position.y < Y1[1]:
            print('[INFO] State: reached_p1')
            return 'reached_p1'

        elif X2[0]< self.position.x < X2[1] and Y2[0] < self.position.y < Y2[1]:
            print('[INFO] State: reached_p2')
            return 'reached_p2'

        elif (X3[0]< self.position.x < X3[1]) and (Y3[0] < self.position.y < Y3[1]):
            print('[INFO] State: reached_p3')
            return 'reached_p3'

        elif (X4[0]< self.position.x < X4[1]) and (Y4[0] < self.position.y < Y4[1]):
            print('[INFO] State: reached_p4')
            return 'reached_p4'

        elif X5[0]< self.position.x < X5[1] and Y5[0] < self.position.y < Y5[1]:
            print('[INFO] State: reached_p5')
            return 'reached_p5'

        elif self.line_info.detected == True:
            
            self.vel.linear.x = self.pid_param["S"]
            self.vel.angular.z = self.pid(self.line_info.error/10)
            #print("after pid robot speed: \n", str(self.vel))
            print('[INFO] State: running Line Follow')
            self.publish()
            
        
        elif self.line_info.detected == False:
            
            self.vel.linear.x = -0.02
            if self.line_info.error >= 0:
                self.vel.angular.z = -0.6
            else:
                self.vel.angular.z = 0.6
            self.publish()
            print("[INFO] State: Searching Line")
            
        
        # else:
        #     self.shutdown()
        #     return 'invalid'

    


m1_outcomes = ['fail','succeed']

class Mission1(State):
    def __init__(self):
        # Your state initialization goes here
        State.__init__(self, outcomes=m1_outcomes)
        

    def execute(self, user_data):
        # Your state execution goes here
        print("[INFO] State: executing Mission1")
        sleep(4)
            
        return 'succeed'
        
        

m2_outcomes = ['fail','succeed']

class Mission2(State):
    def __init__(self):
        # Your state initialization goes here
        State.__init__(self, outcomes=m2_outcomes)

    def execute(self, user_data):
        # Your state execution goes here
        try:
            
            sleep(3)
            return 'succeed'
        except:
            return 'fail'
        

m3_outcomes = ['fail','succeed','running']

class Mission3(State):
    def __init__(self):
        # Your state initialization goes here
        State.__init__(self, outcomes=m3_outcomes)

    def execute(self, user_data):
        # Your state execution goes here
        if sleep(2):
            return 'succeed'

        return 'running'






