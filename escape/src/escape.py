#!/usr/bin/env python
# BEGIN ALL
import os
import sys
import rospy
import smach
import smach_ros
import time
import math
import random
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan, Joy
from ros_numpy import numpify
import numpy as np

class WaitForCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        while g_joy_state is None or g_joy_state.buttons[0] != 1:
            time.sleep(0.01)
        return 'done'


class DrivingForward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['hit', 'goal'])

    def execute(self, userdata):
        while True:
            closest_idx = np.argmin(g_ranges)
            closest = g_ranges[closest_idx]
            if closest < 1.3:
                direction = 1 if random.random() > 0.5 else -1
                while closest < 1.5:
                    drive_turtlebot(0, direction)
                    closest = np.min(g_ranges)
                    time.sleep(0.01)
            if closest < 2.5:
                if closest_idx < len(g_ranges)/2:
                    drive_turtlebot(0.5,1)
                else:
                    drive_turtlebot(0.5,-1)
            else:
                drive_turtlebot(1,0)
            if (g_bumper_state == 1):
                return 'hit'
            time.sleep(0.01)

class BackUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'goal'])
    def execute(self, userdata):
        delay_time = rospy.Time.now() + rospy.Duration(0.5)
        while (rospy.Time.now() < delay_time):
            drive_turtlebot(-1,0)
            time.sleep(0.01)
        turn_turtlebot(np.pi,1)
        return 'done'

class GoalReached(smach.State):
    def __init__(self):
        smach.State.__init__(self)
    def execute(self, userdata):
        drive_turtlebot(0)

def drive_turtlebot(linear_x, angular_z):
    twist = Twist()
    twist.linear.x = linear_x
    twist.angular.z = angular_z
    cmd_vel_pub.publish(twist)

def turn_turtlebot(angle,direction):
    speed = 1
    start_angle = g_angle
    while abs(g_angle - start_angle - angle) > 0.2:
        time.sleep(0.01)
        drive_turtlebot(0,speed*direction)
    drive_turtlebot(0,0)

def turn_turtlebot_exact(angle,direction):
    speed = 1
    start_angle = g_angle
    while abs(g_angle - start_angle - angle) > 0.02 and speed > 0.2:
        while abs(g_angle - start_angle - angle) > 0.02:
            time.sleep(0.01)
            drive_turtlebot(0,speed*direction)
        drive_turtlebot(0,0)
        time.sleep(0.05)
        speed *= 0.5
        direction *= -1
    drive_turtlebot(0,0)

def odom_callback(msg):
    global g_pos
    global g_orientation
    global g_angle
    g_pos = msg.pose.pose.position
    g_orientation = msg.pose.pose.orientation
    quat = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    g_angle = yaw

def bumper_callback(msg):
    print("bump")
    global g_bumper_state
    g_bumper_state = msg.state

def scan_callback(msg):
    global g_ranges
    ranges = np.array(msg.ranges)
    ranges = np.where(np.isnan(ranges), np.inf, ranges)
    g_ranges = ranges

def joy_callback(joy_msg):
    global g_joy_state
    g_joy_state = joy_msg
    if joy_msg.buttons[3] == 1:
        os.system("kill -9 " + str(os.getpid()))

g_ranges = np.array([np.inf])
g_pos = 0 # anything to start
g_orientation = 0 # anything to start
g_bumper_state = 0 # anything to start
g_joy_state = None
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumper_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.Subscriber('joy', Joy, joy_callback)
rospy.init_node('escape')

time.sleep(1) # wait for boot

sm = smach.StateMachine(outcomes=[])
sm.userdata.delay = rospy.Time.now()

with sm:
    smach.StateMachine.add('DISABLED', WaitForCommand(),
                            transitions={'done':'DRIVING FORWARD'})
    smach.StateMachine.add('DRIVING FORWARD', DrivingForward(),
                            transitions={'hit':'BACK UP',
                                         'goal': 'GOAL REACHED'})
    smach.StateMachine.add('BACK UP', BackUp(),
                            transitions={'done':'DRIVING FORWARD',
                                         'goal':'GOAL REACHED'})
    smach.StateMachine.add('GOAL REACHED', GoalReached())

sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
sis.start()

sm.execute()
