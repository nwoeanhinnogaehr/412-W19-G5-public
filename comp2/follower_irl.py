#!/usr/bin/env python
import rospy, cv2, cv_bridge, time
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',
                                          Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                           Twist, queue_size=1)

        # PID state
        self.integral = 0
        self.prev_error = 0

        # PID paramaters
        self.Kp = 1
        self.Ki = 0
        self.Kd = 3

        self.red_visible = 0
        self.red_cutoff = 100000 # number of red points required for detection
        self.stop_time = rospy.get_time()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # color masks
        lower_white = np.array([180, 180, 180]) # RGB
        upper_white = np.array([255, 255, 255])
        lower_red = np.array([80, 180, 0]) # HSV
        upper_red = np.array([255, 255, 255])
        white_mask = cv2.inRange(image, lower_white, upper_white)
        red_mask = cv2.inRange(hsv, lower_red, upper_red)

        # mask out the region we care about
        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        white_mask[0:search_top, 0:w] = 0
        white_mask[search_bot:h, 0:w] = 0
        red_mask[0:search_top, 0:w] = 0
        red_mask[search_bot:h, 0:w] = 0

        white_moments = cv2.moments(white_mask)
        red_moments = cv2.moments(red_mask)

        # unless we are stopped
        if rospy.get_time() > self.stop_time:

            # check for new detection of red line
            if self.red_visible <= 0 and red_moments['m00'] > self.red_cutoff:
                cx = int(red_moments['m10']/red_moments['m00'])
                print('cx',cx)
                print('stopping',red_moments['m00'])
                twist = Twist()
                twist.linear.x = 0
                twist.angular.z = 0
                self.cmd_vel_pub.publish(twist)
                self.stop_time = rospy.get_time() + 10
                self.red_visible = 5
            else:
                # if any white line detected
                if white_moments['m00'] > 0:

                    # if red is still visible, decrement the red cooldown timer
                    if red_moments['m00'] <= self.red_cutoff:
                        self.red_visible -= 1

                    # calculate centroid of white line
                    cx = int(white_moments['m10']/white_moments['m00'])
                    cy = int(white_moments['m01']/white_moments['m00'])

                    # plot circle on centroid
                    cv2.circle(hsv, (cx, cy), 20, (100,255,255), -1)

                    # PID controller
                    err = cx - w/2
                    self.integral += err
                    derivative = (err - self.prev_error)
                    output = self.Kp * err + self.Ki * self.integral + self.Kd * derivative
                    self.prev_error = err
                    twist = Twist()
                    twist.linear.x = 0.5
                    twist.angular.z = -float(output) / 100
                    self.cmd_vel_pub.publish(twist)


        # display debug image
        cv2.imshow("window", hsv)
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
