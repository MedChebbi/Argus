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
from states import LineFollow, Mission1, Mission2


class main():
    def __init__(self):
        rospy.init_node("argus_test_node")

        self.pub = rospy.Publisher("control/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        self.vel = Twist()
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x= 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0
        self.auto_mode = Bool()
        self.auto_mode = False
        
        self.publish()
        rospy.Subscriber('/teleop/command', Bool, self.teleCallback)
        rospy.Subscriber('/teleop/cmd_vel', Twist, self.callback)

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        outcomes = ['preempted','sm_finished','sm_running']
       

        self.get_init_position()

        
        #rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Third State Machine
        self.sm = StateMachine(outcomes=outcomes)

        # self.sm.userdata.sm_position = self.position    
        
        with self.sm:
            StateMachine.add('LINEFOLLOW', LineFollow("/detected_line/info", LineInfo),
                            transitions={'reached_p1':'MISSION1', 'reached_p2':'MISSION2', 'reached_p3':'sm_running',
                                        'reached_p4':'sm_running', 'reached_p5':'sm_running',
                                        'preempted':'preempted'},
                            )
        
            StateMachine.add('MISSION1', Mission1(),
                            transitions={'succeed':'sm_finished','fail':'sm_finished'})

            StateMachine.add('MISSION2', Mission2(),
                            transitions={'succeed':'LINEFOLLOW','fail':'sm_finished'})

        
        #sis = IntrospectionServer('smach_server', self.sm, '/SM_ROOT')
        #sis.start()
        while not rospy.is_shutdown():
            if self.auto_mode == True:
                outcome = self.sm.execute()
                print(outcome)
            else:
                print("[INFO] Manual mode")
                print("robot speed: \n", str(self.vel))
            self.publish()
        
        #sis.stop()
        

    def publish(self):
        self.pub.publish(self.vel)
        self.rate.sleep()
        print("Main publishing")


    def callback(self, data):
        self.vel = data


    def teleCallback(self, data):
        print(data)
        self.auto_mode = data.data
        print(self.auto_mode)


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.sm.request_preempt()
        rospy.sleep(1)


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
        self.sm.userdata.position = self.position


if __name__=="__main__":
    
    try:
        main()
    except rospy.ROSInterruptException as e:
        rospy.logwarn(e)
