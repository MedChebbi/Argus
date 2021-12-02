#!/usr/bin/env python3

import smach
import smach_ros

from controller import PID

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from argus_msgs.msg import LineInfo
from argus_control.cfg import pidParamConfig
from dynamic_reconfigure.server import Server


P1 = [[0.56,0.72],[-0.08,0.08]]
P2 = [[0.9,1],[-0.08,0.08]]
P3 = [[0.9,1],[-0.65,-0.6]]
P4 = [[0.56,0.72],[-0.65,-0.6]]
P5 = [[0.2,0.3],[-0.65,-0.6]]

line_follow_outcomes = ['reached_p1', 'reached_p2', 'reached_p3', 'reached_p4', 'reached_p5', 'running', 'failed']
line_follow_inputs = ['position']
line_follow_outputs = ['vel']

class LineFollow(smach.State):
    def __init__(self):
        # Your state initialization goes here
        smach.State.__init__(self, outcomes=line_follow_outcomes,
                             input_keys = line_follow_inputs,
                             output_keys = line_follow_outputs)
        
        self.vel = Twist()
        
        self.line_info = LineInfo()
        self.pid_param = {"KP": 0.1, "KI": 0, "KD": 0.01, "S": 0.1}
        srv = Server(pidParamConfig, self.reconfig)
        self.Kp, self.Ki, self.Kd = self.pid_param["KP"], self.pid_param["KI"], self.pid_param["KD"]

        self.pid = PID(self.Kp, self.Ki, self.Kd,sample_time=0.01,setpoint=0,output_limits=(-1.5, 1.5))
        rospy.Subscriber("/detected_line/info", LineInfo , self.lineCallback)
    

    def execute(self, user_data):
        # Your state execution goes here

        print(self.line_info.error)
        if self.line_info.detected == True:
            
            user_data.vel.linear.x = self.pid_param["S"]
            user_data.vel.angular.z = self.pid(user_data.line_info.error/10)
            print("after pid robot speed: \n", str(user_data.vel))
       
            return 'running'
            
        elif P1[0][0] < user_data.position.x < P1[0][1] and P1[1][0] < user_data.position.y < P1[1][1]:
            return 'reached_p1'

        elif P2[0][0] < user_data.position.x < P2[0][1] and P2[1][0] < user_data.position.y < P2[1][1]:
            return 'reached_p2'

        elif P3[0][0] < user_data.position.x < P3[0][1] and P3[1][0] < user_data.position.y < P3[1][1]:
            return 'reached_p3'

        elif P4[0][0] < user_data.position.x < P4[0][1] and P4[1][0] < user_data.position.y < P4[1][1]:
            return 'reached_p4'

        elif P5[0][0] < user_data.position.x < P5[0][1] and P5[1][0] < user_data.position.y < P5[1][1]:
            return 'reached_p5'

        elif self.line_info.detected == False:
            
            user_data.vel.linear.x = -0.02
            if self.line_info.error >= 0:
                user_data.vel.angular.z = -0.6
            else:
                user_data.vel.angular.z = 0.6
            print("no line in auto mode \n robot speed: \n", str(user_data.vel))
            return 'failed'

    def reconfig(self, config, level):
        self.pid_param = config
        print(self.pid_param)
        return config


    def lineCallback(self, data):
        self.line_info = data
    
        
################################################################################################################################



def run_sm():
    while not rospy.is_shutdown():
        print('out: ',line_info)
        if auto_mode == True:
            
            # Execute SMACH plan
            # outcome = sm.execute()
            # vel = sm.userdata.vel
            print("hey")

                
        else:
            print("[INFO] Manual mode")
            print("robot speed: \n", str(vel))
        publish()
        rospy.on_shutdown(stop)

def publish(vel):
    pub = rospy.Publisher("control/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10)
    pub.publish(vel)
    rate.sleep()
    print("publishing")

def stop(vel, pub):
    auto_mode == False
    vel.linear.x = 0
    vel.angular.z = 0
    publish(vel)
    print("shutting down")


def get_init_position():
    data_odom = None
    while data_odom is None:
        try:
            data_odom = rospy.wait_for_message('/odom', Odometry, timeout = 1)
        except:
            rospy.loginfo("Current odom not ready yet, retrying for setting up init position")
    current_position = Point()
    current_position.x = data_odom.pose.pose.position.x
    current_position.y = data_odom.pose.pose.position.y
    current_position.z = data_odom.pose.pose.position.z
    return current_position

def odom_callback(msg):
    NewPosition = msg.pose.pose.position
    sm.userdata.position = NewPosition

def callback(data):
    vel = data


def teleCallback(data):
    print(data)
    auto_mode = data.data
    print(auto_mode)



def main():
    
    rospy.init_node("argus_head_node")

    pub = rospy.Publisher("control/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10)
    vel = Twist()
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x= 0
    vel.angular.y = 0
    vel.angular.z = 0

    auto_mode = Bool()
    auto_mode = True
            
    pos = get_init_position()

    publish(vel)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/teleop/command', Bool, teleCallback)
    rospy.Subscriber('/teleop/cmd_vel', Twist, callback)


    line_follow_inputs = ['position']
    line_follow_outputs = ['vel']
    sm = smach.StateMachine(outcomes=['reached_p1', 'reached_p2', 'reached_p3', 'reached_p4', 'reached_p5', 'running', 'failed'],
                                input_keys = line_follow_inputs,
                                output_keys = line_follow_outputs)


    sm.userdata.position = pos
    # remapping={'vel':'sm_vel', 'pid','sm_pid'}
    with sm:
        smach.StateMachine.add('LINEFOLLOW', LineFollow(),
                                transitions={'reached_p1':'reached_p1',
                                'reached_p2':'reached_p2',
                                'reached_p3':'reached_p3',
                                'reached_p4':'reached_p4',
                                'reached_p5':'reached_p5',
                                'running':'running',
                                'failed':'failed'},
                                )

    outcome = sm.execute()
    vel = sm.userdata.vel
    print(outcome)


if __name__=="__main__":
    
    try:
        main()
    except rospy.ROSInterruptException as e:
        rospy.logwarn(e)

