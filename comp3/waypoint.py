#!/usr/bin/env python

import rospy
import actionlib
import time
import pickle
import os

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped

print(os.getcwd())

pose = None
waypoints = []
def acml_callback(msg):
    global pose
    pose = msg.pose

def joy_callback(joy_msg):
    global waypoints
    if joy_msg.buttons[0] == 1:
        waypoints.append(pose)
        print("SAVE ",pose)
        with open("waypoints.pickle", "wb") as f:
            pickle.dump(waypoints,f)

if __name__ == '__main__':
    rospy.init_node('waypoint')

    rospy.Subscriber('joy', Joy, joy_callback)
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, acml_callback)

    while True:
        time.sleep(0.01)
