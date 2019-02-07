#!/usr/bin/env python

"""
    follower.py - Version 1.1 2013-12-20

    Follow a "person" by tracking the nearest object in x-y-z space.

    Based on the follower application by Tony Pratkanis at:

    http://ros.org/wiki/turtlebot_follower

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from roslib import message
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Joy
from math import copysign
from ros_numpy import numpify
import ros_numpy.point_cloud2
import numpy as np
import time

class Follower():
    def __init__(self):
        rospy.init_node("follower")

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)

        # The dimensions (in meters) of the box in which we will search
        # for the person (blob). These are given in camera coordinates
        # where x is left/right,y is up/down and z is depth (forward/backward)
        self.min_x = rospy.get_param("~min_x", -0.2)
        self.max_x = rospy.get_param("~max_x", 0.2)
        self.min_y = rospy.get_param("~min_y", -0.3)
        self.max_y = rospy.get_param("~max_y", 0.3)
        self.max_z = rospy.get_param("~max_z", 1.2)

        # The goal distance (in meters) to keep between the robot and the person
        self.goal_z = rospy.get_param("~goal_z", 1.3)

        # How far away from the goal distance (in meters) before the robot reacts
        self.z_threshold = rospy.get_param("~z_threshold", 0.05)

        # How far away from being centered (x displacement) on the person
        # before the robot reacts
        self.x_threshold = rospy.get_param("~x_threshold", 0.05)

        # How much do we weight the goal distance (z) when making a movement
        self.z_scale = rospy.get_param("~z_scale", 1.0)

        # How much do we weight left/right displacement of the person when making a movement
        self.x_scale = rospy.get_param("~x_scale", 2.0)

        # The maximum rotation speed in radians per second
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 2.0)

        # The minimum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 1)

        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 1)

        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.25)

        # Slow down factor when stopping
        self.slow_down_factor = rospy.get_param("~slow_down_factor", 0)

        # Initialize the movement command
        self.move_cmd = Twist()

        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to the point cloud
        self.depth_subscriber = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.set_cmd_vel, queue_size=1)

        # Nothing detected to start
        self.objects = []

        rospy.loginfo("Subscribing to point cloud...")

        # Wait for the pointcloud topic to become available
        rospy.wait_for_message('/camera/depth_registered/points', PointCloud2)

        self.joy_state = None
        rospy.Subscriber('joy', Joy, self.joy_callback)
        while self.joy_state is None or self.joy_state.buttons[0] != 1:
            time.sleep(0.01)

        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)


        rospy.loginfo("Ready to follow!")


        self.prev_x = np.nan
        self.prev_y = np.nan
        self.prev_z = np.nan
        self.prev_size = np.nan

    def set_cmd_vel(self, msg):
        # convert msg to 6x480x640 numpy array (x,y,z,r,g,b)
        pts = numpify(msg)
        pts = ros_numpy.point_cloud2.split_rgb_field(pts)
        pts = np.array([pts['x'], pts['y'], pts['z'], pts['r'], pts['g'], pts['b']])

        objstats = []
        for obj in self.objects:
            print("Testing object", obj)

            objpts = pts[:,:,pts.shape[2]-obj[1]:pts.shape[2]-obj[0]]
            print("after laser filter:", objpts.shape)
            size = obj[1] - obj[0]

            # filter NaNs
            objpts = objpts[:, (1 - (np.isnan(objpts[0]) | np.isnan(objpts[1]) | np.isnan(objpts[2]))).astype(bool)]
            print("after NaN filter:", objpts.shape)

            #filter_cond = np.all([pts[0] < self.max_x, pts[0] > self.min_x, -pts[1] < self.max_y, -pts[1] > self.min_y, pts[2] < self.max_z], axis=0)

            # only filter Y
            filter_cond = np.all([-objpts[1] < self.max_y, -objpts[1] > self.min_y], axis=0)
            objpts = objpts[:, filter_cond]

            n = objpts.shape[1]
            if n:
                cx = np.sum(objpts[0])/n
                cy = np.sum(objpts[1])/n
                cz = np.sum(objpts[2])/n
                color = np.sum(objpts[3]+objpts[4]+objpts[5])/n
                if np.isnan(self.prev_x):
                    dist_to_prev = np.nan
                    score = color
                else:
                    dist_to_prev = np.sqrt((cx - self.prev_x)**2 + (cy - self.prev_y)**2 + (cz - self.prev_z)**2)
                    score = (color/100) + dist_to_prev + (size - self.prev_size) / 640 + size / 400 + cz/4
                print(n,cx,cy,cz,color, dist_to_prev, score)
                objstats.append([n,cx,cy,cz,score,size])
            else:
                print("No points!")

        if len(objstats):
            objstats = np.array(objstats).T

            lowest_score = np.argmin(objstats[4])
            n = objstats[0,lowest_score]
            x = objstats[1,lowest_score]
            y = objstats[2,lowest_score]
            z = objstats[3,lowest_score]
            size = objstats[5,lowest_score]

            # If we have points, compute the centroid coordinates
            if n > 10:
                self.prev_x = x
                self.prev_y = y
                self.prev_z = z
                self.prev_size = size

                # Check our movement thresholds
                if (abs(z - self.goal_z) > self.z_threshold):
                    # Compute the angular component of the movement
                    linear_speed = (z - self.goal_z) * self.z_scale

                    # Make sure we meet our min/max specifications
                    self.move_cmd.linear.x = copysign(max(self.min_linear_speed,
                                            min(self.max_linear_speed, abs(linear_speed))), linear_speed)
                else:
                    self.move_cmd.linear.x *= self.slow_down_factor

                if (abs(x) > self.x_threshold):
                    # Compute the linear component of the movement
                    angular_speed = -x * self.x_scale

                    # Make sure we meet our min/max specifications
                    self.move_cmd.angular.z = copysign(max(self.min_angular_speed,
                                            min(self.max_angular_speed, abs(angular_speed))), angular_speed)
                else:
                    # Stop the rotation smoothly
                    self.move_cmd.angular.z *= self.slow_down_factor

            else:
                # Stop the robot smoothly
                self.move_cmd.linear.x *= self.slow_down_factor
                self.move_cmd.angular.z *= self.slow_down_factor

            # Publish the movement command
            self.cmd_vel_pub.publish(self.move_cmd)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        # Unregister the subscriber to stop cmd_vel publishing
        self.depth_subscriber.unregister()
        rospy.sleep(1)

        # Send an emtpy Twist message to stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def scan_callback(self, msg):
        self.objects = self.get_objects(msg.ranges)
        print(self.objects)

    def get_objects(self, ranges):
        objects = []
        in_object = False
        current_object = []
        nan_count = 0
        dist_threshold = 0.3
        max_nans_in_a_row = 4
        for i in range(0,len(ranges)):
            if not np.isnan(ranges[i]):
                if not in_object:
                    current_object.append(i)
                    in_object = True
                nan_count = 0
            else:
                if in_object and (abs(ranges[i] - ranges[i-1]) > dist_threshold or nan_count > max_nans_in_a_row):
                    current_object.append(i-max_nans_in_a_row)
                    objects.append(current_object)
                    current_object = []
                    in_object = False
                nan_count += 1
        if in_object:
            current_object.append(len(ranges)-1)
            objects.append(current_object)

        return objects

    def joy_callback(self, joy_msg):
        self.joy_state = joy_msg


if __name__ == '__main__':
    try:
        Follower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
rospy.loginfo("Follower node terminated.")
