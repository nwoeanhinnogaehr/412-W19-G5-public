#!/usr/bin/env python
import os
import rospy, cv2, cv_bridge, time, smach, smach_ros, math, sys
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import shape_detect
import subprocess

class Following(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn', 'atL2'])

        self.bridge = cv_bridge.CvBridge()
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

        # turn off led lights
        led1_pub.publish(Led.BLACK)
        led2_pub.publish(Led.BLACK)

    def execute(self, userdata):
        while(True):
            global global_inx, target, g_range_ahead
            #print(global_inx)
            rospy.wait_for_message('/camera/rgb/image_raw', Image)
            # color masks
            lower_white = np.array([200, 200, 200]) # RGB
            upper_white = np.array([257, 257, 257])
            lower_red = np.array([80, 180, 0]) # HSV
            upper_red = np.array([255, 255, 255])
            white_mask = cv2.inRange(image, lower_white, upper_white)
            red_mask = cv2.inRange(hsv, lower_red, upper_red)

            # mask out the region we care about
            h, w, d = image.shape
            search_top = 7*h/8
            search_bot = 7*h/8 + 20
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
                    twist.linear.x = 0.5
                    # delay stop or not
                    if should_delay():
                        if global_inx != 3:
                            time_to_delay = 0.95
                        else:
                            time_to_delay = 1.2
                        self.stop_time = rospy.get_time() + time_to_delay
                        while(rospy.get_time() < self.stop_time):
                            cmd_vel_pub.publish(twist)
                    twist.linear.x = 0
                    twist.angular.z = 0
                    cmd_vel_pub.publish(twist)
                    self.stop_time = rospy.get_time() + 1
                    self.red_visible = 5
                    # determine what to do at this location
                    if global_inx in [0, 2, 3, 6, 8, 10]:
                        target = -math.pi/2 # turn left 90 degrees
                        return 'turn'
                    if global_inx in [1, 4, 5]:
                        global_inx += 1 # at a stopline, pause then continue
                    if global_inx == 12:
                        led1_pub.publish(Led.BLACK)
                        led2_pub.publish(Led.BLACK)
                        sys.exit() # done loop


                else:
                    # if any white line detected, follow it
                    if white_moments['m00'] > 0:

                        # if red is still visible, decrement the red cooldown timer
                        if red_moments['m00'] <= self.red_cutoff:
                            self.red_visible -= 1

                        # calculate centroid of white line
                        cx = int(white_moments['m10']/white_moments['m00'])
                        cy = int(white_moments['m01']/white_moments['m00'])

                        # plot circle on centroid
                        cv2.circle(image, (cx, cy), 20, (100,255,255), -1)

                        # PID controller
                        err = cx - w/2
                        self.integral += err
                        derivative = (err - self.prev_error)
                        output = self.Kp * err + self.Ki * self.integral + self.Kd * derivative
                        self.prev_error = err
                        twist = Twist()
                        twist.linear.x = 0.5
                        twist.angular.z = -float(output) / 100
                        cmd_vel_pub.publish(twist)
                    else:
                        print("NO LINE")
                        drive_turtlebot(0.2,0)
            # display debug image
            cv2.imshow("window", image)
            cv2.waitKey(3)

            # use laserscan to detect location 2
            if(global_inx == 3 and g_range_ahead < 1.2):
                drive_turtlebot(0, 0)
                return 'atL2'



class Turning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['atL1', 'atL3', 'follow'])
    def execute(self, userdata):
        global global_inx
        turn_turtlebot(target)

        # determine what to do next based on location
        if global_inx == 0:
            return 'atL1'
        elif global_inx in [1,2,3,7,9,11]:
            if global_inx != 1:
                global_inx += 1
            return 'follow'
        elif global_inx in [6,8,10]:
            return 'atL3'


class Location1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
    def execute(self, userdata):
        cv2.imshow("window", image)
        global target, global_inx
        drive_turtlebot(0, 0)
        target = math.pi/2 # turn right away from location1
        # wait for turtlebot to stop
        stop_time = rospy.get_time() + 1
        while(rospy.get_time() < stop_time):
            pass
        rospy.wait_for_message('/scan', LaserScan)
        # Scan objects with shape detector
        num_of_objects = len(shape_detect.detect(hsv, "red", 1000))
        if num_of_objects == 4: # HACK
            num_of_objects = 3
        #num_of_objects = len(get_objects(obj_ranges))
        show_count(num_of_objects) # display on lights and sound
        print("NUMBER OF OBJECTS DETECTED AT LOCATION1: ", num_of_objects)
        global_inx += 1
        return 'done'

class Location2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
    def execute(self, userdata):
        global target, global_inx, g_green_shape
        turn_turtlebot(-math.pi/10) # correct angle a bit
        # wait for turtlebot to stop
        stop_time = rospy.get_time() + 0.5
        while(rospy.get_time() < stop_time):
            pass
        # detect shapes
        green_shapes = shape_detect.detect(hsv, "green", 1000)
        g_green_shape = green_shapes
        red_shapes = shape_detect.detect(hsv, "red", 1000)
        num = min(3,len(green_shapes) + len(red_shapes)) # HACK
        show_count(num) # display on lights and sound
        print("green", green_shapes)
        print("red", red_shapes)
        print("NUMBER OF OBJECTS DETECTED AT LOCATION2: ", num)
        global_inx -= 1
        target = math.pi #spin around at location2
        return 'done'

class Location3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
    def execute(self, userdata):
        global target, global_inx
        target = math.pi/2 # rotations back from location 3
        turn_turtlebot(-math.pi/10) # fix angle
        # wait for turtlebot to stop
        stop_time = rospy.get_time() + 0.5
        while(rospy.get_time() < stop_time):
            pass
        # detect shape
        shapes = shape_detect.detect(hsv, "red", 7000)
        print("LOCATION3 SHAPES", shapes)
        if g_green_shape[0] in shapes: # match with green shape from location2
            path=os.path.dirname(os.path.realpath(__file__))
            g_mpv = subprocess.Popen(['aplay', path+'/'+'checkpoint-hit.wav'])

        # back up a bit so we don't miss the next line
        turn_turtlebot(math.pi/2)
        stop_time = rospy.get_time() + 0.7
        while(rospy.get_time() < stop_time):
            drive_turtlebot(-0.5,0)

        global_inx += 2
        return 'done'

def should_delay():
    if global_inx in [1,4,5,9]:
        return False
    else:
        return True

# play a sound and show count on LEDs
def show_count(num_of_objects):
    global g_mpv
    if num_of_objects <= 3:
        path=os.path.dirname(os.path.realpath(__file__))
        g_mpv = subprocess.Popen(['aplay', path+'/'+['0.wav','1.wav','2.wav','3.wav'][num_of_objects]])
    if num_of_objects == 1:
        led2_pub.publish(Led.ORANGE)
    elif num_of_objects == 2:
        led1_pub.publish(Led.ORANGE)
    elif num_of_objects == 3:
        led1_pub.publish(Led.ORANGE)
        led2_pub.publish(Led.ORANGE)
    else:
        led1_pub.publish(Led.RED)

def drive_turtlebot(linear_x, angular_z):
    twist = Twist()
    twist.linear.x = linear_x
    twist.angular.z = angular_z
    cmd_vel_pub.publish(twist)

# turn by angle, positive=clockwise
def turn_turtlebot(angle):
    speed = 1.5
    start_angle = yaw
    while abs(abs(yaw - start_angle) - abs(angle)) > 0.15:
        time.sleep(0.01)
        drive_turtlebot(0,speed*(-1 if angle > 0 else 1))
    drive_turtlebot(0,0)

# get list of objects in laser scan
def get_objects(ranges):
    objects = []
    in_object = False
    current_object = []
    nan_count = 0
    dist_threshold = 0.3
    max_nans_in_a_row = 4
    size_of_object = 0
    for i in range(0,len(ranges)):
        if ranges[i] > 0.9 or ranges[i] < 0.5:
            ranges[i] = np.nan
    for i in range(0,len(ranges)):
        if not np.isnan(ranges[i]):
            if not in_object:
                current_object.append(i)
                in_object = True
            nan_count = 0
            size_of_object += 1
        else:
            if in_object and (abs(ranges[i] - ranges[i-1]) > dist_threshold or nan_count > max_nans_in_a_row):
                if size_of_object > 60:
                    current_object.append(i-max_nans_in_a_row)
                    objects.append(current_object)
                size_of_object = 0
                current_object = []
                in_object = False
            nan_count += 1
    if in_object:
        current_object.append(len(ranges)-1)
        objects.append(current_object)

    return objects

def image_callback(msg):
    global bridge, image, hsv
    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

def scan_callback(msg):
    global g_range_ahead
    #global obj_ranges, ranges
    #obj_ranges = np.array(msg.ranges)
    #print(len(get_objects(obj_ranges)))
    #print(obj_ranges)
    ranges = np.array(msg.ranges)
    where_are_NaNs = np.isnan(ranges)
    ranges[where_are_NaNs] = 3
    g_range_ahead = np.average(ranges)

def odom_callback(msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

# set odom varibles
roll = pitch = yaw = 0.0
target = -math.pi/2 # first turn at location1
kp=0.8

# set scan variables
g_range_ahead = 10 # anything to start
#ranges = []
#obj_ranges =[]

# set image variables
bridge = cv_bridge.CvBridge()
image = []
hsv = []

global_inx = 0

# set subscribers and publishers
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
led1_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=10)
led2_pub = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=10)

rospy.init_node('counter')

sm = smach.StateMachine(outcomes=[])

with sm:
    smach.StateMachine.add('FOLLOWING', Following(),
                          transitions={'turn':'TURNING',
                                       'atL2':'LOCATION2'})
    smach.StateMachine.add('TURNING', Turning(),
                          transitions={'atL1':'LOCATION1',
                                       'atL3': 'LOCATION3',
                                       'follow' : 'FOLLOWING'})
    smach.StateMachine.add('LOCATION1', Location1(),
                          transitions={'done':'TURNING'})
    smach.StateMachine.add('LOCATION2', Location2(),
                          transitions={'done':'TURNING'})
    smach.StateMachine.add('LOCATION3', Location3(),
                          transitions={'done':'FOLLOWING'})

sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
sis.start()

# wait for camera to auto adjust
time.sleep(2)

sm.execute()
