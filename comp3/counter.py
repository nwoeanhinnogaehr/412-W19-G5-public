#!/usr/bin/env python
import os
import rospy, cv2, cv_bridge, time, smach, smach_ros, math, sys
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sound_play.libsoundplay import SoundClient
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import shape_detect
import subprocess
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy
import pickle
import rospkg
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

turtlebot_speed = 0.3

rospack = rospkg.RosPack()
waypoints = pickle.load(open(rospack.get_path('comp3')+"/waypoints.pickle"))
print(waypoints)

class Following(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn', 'atL2', 'atL4'])

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

        rospy.wait_for_message('/usb_cam/image_raw', Image)

    def execute(self, userdata):
        while(True):
            global global_inx, target, g_range_ahead
            #print(global_inx)

            # color masks
            lower_white = np.array([0, 0, 150]) # HSV
            upper_white = np.array([257, 60, 257])
            white_mask = cv2.inRange(logi_hsv, lower_white, upper_white)
            red_lower = np.array([0, 130, 0]) # HSV
            red_upper = np.array([20, 258, 258])
            red_lower2 = np.array([160, 130, 0]) # HSV
            red_upper2 = np.array([258, 258, 258])
            red_mask1 = cv2.inRange(logi_hsv, red_lower, red_upper)
            red_mask2 = cv2.inRange(logi_hsv, red_lower2, red_upper2)
            red_mask = red_mask1 | red_mask2

            # mask out the region we care about
            h, w, d = logi_image.shape
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
                    twist.linear.x = turtlebot_speed
                    # delay stop or not
                    if should_delay():
                        time_to_delay = 1.5
                        self.stop_time = rospy.get_time() + time_to_delay
                        while(rospy.get_time() < self.stop_time):
                            cmd_vel_pub.publish(twist)
                    twist.linear.x = 0
                    twist.angular.z = 0
                    cmd_vel_pub.publish(twist)
                    self.stop_time = rospy.get_time() + 1
                    self.red_visible = 5
                    if global_inx == 4:
                        global_inx += 1 # at a stopline, pause then continue
                        return 'atL4'
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
                    # if any white line detected
                    if white_moments['m00'] > 0:

                        # if red is still visible, decrement the red cooldown timer
                        if red_moments['m00'] <= self.red_cutoff:
                            self.red_visible -= 1

                        # calculate centroid of white line
                        cx = int(white_moments['m10']/white_moments['m00'])
                        cy = int(white_moments['m01']/white_moments['m00'])

                        # plot circle on centroid
                        cv2.circle(logi_image, (cx, cy), 20, (100,255,255), -1)

                        # PID controller
                        err = cx - w/2
                        self.integral += err
                        derivative = (err - self.prev_error)
                        output = self.Kp * err + self.Ki * self.integral + self.Kd * derivative
                        self.prev_error = err
                        twist = Twist()
                        twist.linear.x = turtlebot_speed
                        twist.angular.z = -float(output) / 100
                        cmd_vel_pub.publish(twist)
                    else:
                        print("NO LINE")
                        drive_turtlebot(0.2,0)

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
        global target, global_inx
        drive_turtlebot(0, 0)
        target = math.pi/2 # turn right away from location1
        stop_time = rospy.get_time() + 1
        while(rospy.get_time() < stop_time):
            pass
        rospy.wait_for_message('/scan', LaserScan)
        # Scan objects with LaserScan
        # num_of_objects = min(3,len(shape_detect.detect(asus_hsv, "red", 1200)))
        #num_of_objects = len(shape_detect.detect(asus_hsv, "red", 1200)) -1
        num_of_objects = len(get_objects(obj_ranges))
        show_count(num_of_objects)
        print("NUMBER OF OBJECTS DETECTED AT LOCATION1: ", num_of_objects)
        global_inx += 1
        return 'done'

class Location2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
    def execute(self, userdata):
        global target, global_inx, g_green_shape
        turn_turtlebot(-math.pi/10)
        stop_time = rospy.get_time() + 2
        while(rospy.get_time() < stop_time):
            pass
        green_shapes = shape_detect.detect(asus_hsv, "green", 1000)
        g_green_shape = green_shapes
        red_shapes = shape_detect.detect(asus_hsv, "red", 1000)
        num = min(3,len(green_shapes) + len(red_shapes))
        show_count(num)
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
        global target, global_inx, g_green_shape
        target = math.pi/2 # rotations back from location 3
        turn_turtlebot(-math.pi/10 +0.3)


        # back up a bit so we don't miss the next line
        stop_time = rospy.get_time() + 0.8
        while(rospy.get_time() < stop_time):
            drive_turtlebot(-0.2,0)
        stop_time = rospy.get_time() + 1.5
        while(rospy.get_time() < stop_time):
            pass

        shapes = shape_detect.detect(asus_hsv, "red", 4000)
        try:
            if g_green_shape[0] in shapes:
                path=os.path.dirname(os.path.realpath(__file__))
                #g_mpv = subprocess.Popen(['aplay', path+'/'+'checkpoint-hit.wav'])
                sound_client.playWave(path + '/sounds/' + 'checkpoint-hit.wav')
                g_green_shape = []
        except:
            pass
        print("LOCATION3 SHAPES", shapes)

        stop_time = rospy.get_time() + 0.8
        while(rospy.get_time() < stop_time):
            drive_turtlebot(0.2,0)
        turn_turtlebot(math.pi/2)
        stop_time = rospy.get_time() + 0.5
        while(rospy.get_time() < stop_time):
            drive_turtlebot(-0.2,0)
        # stop_time = rospy.get_time() + 0.7
        # while(rospy.get_time() < stop_time):
        #     drive_turtlebot(-turtlebot_speed,0)

        global_inx += 2
        return 'done'

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose = pose.pose
    return goal_pose


class Location4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
    def execute(self, userdata):
        # drive until just in front of red line
        drive_time = rospy.get_time() + 0.2
        while(rospy.get_time() < drive_time):
            drive_turtlebot(0.2,0)

        # set initial pose
        init_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)
        stop_time = rospy.get_time() + 2
        while(rospy.get_time() < stop_time):
            pose = PoseWithCovarianceStamped()
            pose.pose.pose.orientation.w = 1
            pose.pose.pose.position.y = -0.1
            for i in range(6):
                pose.pose.covariance[i*6+i] = 0.01
            init_pose.publish(pose)
            time.sleep(0.01)

        i = 0
        for waypoint in waypoints:
            goal = goal_pose(waypoint)
            self.client.send_goal(goal)
            self.client.wait_for_result()
            if (i > 1 and i != len(waypoints)-1):
                scan_front(i)
            i += 1
        return 'done'

def scan_front(idx):

    led1_pub.publish(Led.BLACK)
    led2_pub.publish(Led.BLACK)

    # back up
    stop_time = rospy.get_time() + 0.9
    while(rospy.get_time() < stop_time):
        drive_turtlebot(-0.2,0)

    # wait
    stop_time = rospy.get_time() + 2
    while(rospy.get_time() < stop_time):
        pass

    # respresents what signal to send
    result = 0

    # scan
    red_shapes = shape_detect.detect(asus_hsv, "red", 1000)
    if markers:
        print("MARKER DETECTED: " + str(markers[0]))
        result = 3
    elif red_shapes:
        print("RED SHAPE DETECTED: " + str(red_shapes[0]))
        result = 1
        try:
            if red_shapes[0] == g_green_shape[0]:
                result = 2
        except:
            pass
    elif idx == [7,6,5,4,3,9,8,2][5]:
        result = 4
    else:
        print("EMPTY PARKING SPOT")

    # move forward
    stop_time = rospy.get_time() + 0.9
    while(rospy.get_time() < stop_time):
        drive_turtlebot(0.2,0)

    # send out result message with led or soundplay
    path=os.path.dirname(os.path.realpath(__file__))
    if result == 4:
        # empty
        led1_pub.publish(Led.RED)
        led2_pub.publish(Led.RED)
        sound_client.playWave(path + '/sounds/' + 'cow1.wav')
    elif result == 1:
        # shape
        led1_pub.publish(Led.ORANGE)
        led2_pub.publish(Led.ORANGE)
        sound_client.playWave(path + '/sounds/' + 'cat_meow_x.wav')
    elif result == 2:
        # matched with green shape
        led1_pub.publish(Led.ORANGE)
        led2_pub.publish(Led.GREEN)
        sound_client.playWave(path + '/sounds/' + 'checkpoint-hit.wav')
    elif result == 3:
        # ar tag
        led1_pub.publish(Led.GREEN)
        led2_pub.publish(Led.GREEN)
        sound_client.playWave(path + '/sounds/' + 'chicken.wav')

    # wait
    stop_time = rospy.get_time() + 3
    while(rospy.get_time() < stop_time):
        pass

    stop_time = rospy.get_time() + 1.2
    while(rospy.get_time() < stop_time):
        drive_turtlebot(-0.2,0)

def should_delay():
    if global_inx in [1,4,5,9]:
        return False
    else:
        return True

def show_count(num_of_objects):
    led1_pub.publish(Led.BLACK)
    led2_pub.publish(Led.BLACK)
    #global g_mpv
    if num_of_objects <= 3:
        path=os.path.dirname(os.path.realpath(__file__))
        #g_mpv = subprocess.Popen(['aplay', path+'/'+['0.wav','1.wav','2.wav','3.wav'][num_of_objects]])
        sound_client.playWave(path + '/sounds/' + ['0.wav','1.wav','2.wav','3.wav'][num_of_objects])
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

def turn_turtlebot(angle):
    speed = 1.2
    start_angle = yaw
    while abs(abs(yaw - start_angle) - abs(angle)) > 0.15:
        time.sleep(0.01)
        drive_turtlebot(0,speed*(-1 if angle > 0 else 1))
    drive_turtlebot(0,0)

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

def logi_image_callback(msg):
    global logi_bridge, logi_image, logi_hsv
    logi_image = logi_bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    logi_hsv = cv2.cvtColor(logi_image, cv2.COLOR_BGR2HSV)
    cv2.imshow("window1", asus_image)
    cv2.waitKey(3)

def asus_image_callback(msg):
    global asus_bridge, asus_image, asus_hsv
    asus_image = asus_bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    asus_hsv = cv2.cvtColor(asus_image, cv2.COLOR_BGR2HSV)
    # cv2.imshow("window2", asus_image)
    # cv2.waitKey(3)

def scan_callback(msg):
    global g_range_ahead
    global obj_ranges, ranges
    obj_ranges = np.array(msg.ranges)
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

def ar_callback(event):
    global markers
    markers = []
    for marker in event.markers:
        if marker.id not in [2,3,4]:
            continue
        o = marker.pose.pose.orientation
        olist = [o.x, o.y, o.z, o.w]
        (roll, pitch, yaw) = euler_from_quaternion(olist)
        p = marker.pose.pose.position
        markers.append((marker.id, p.x, p.y, p.z, yaw))
        #print(roll, pitch, yaw)
    print(markers)

# set odom varibles
roll = pitch = yaw = 0.0
target = -math.pi/2 # first turn at location1
kp=0.8

# set scan variables
g_range_ahead = 10 # anything to start
#ranges = []
obj_ranges =[]

# set logi_image variables
logi_bridge = cv_bridge.CvBridge()
logi_image = []
logi_hsv = []

# set asus_image variables
asus_bridge = cv_bridge.CvBridge()
asus_image = []
asus_hsv = []

# set variable to store visable artags
markers = []

global_inx = 4

# set subscribers and publishers
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
logi_image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, logi_image_callback)
asus_image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, asus_image_callback)
ar_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback)
odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
led1_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=10)
led2_pub = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=10)

rospy.init_node('counter')

rospy.set_param("/move_base/global_costmap/inflation_layer/inflation_radius", 0.1)
rospy.set_param("/move_base/local_costmap/inflation_layer/inflation_radius", 0.1)
rospy.set_param("/move_base/DWAPlannerROS/xy_goal_tolerance", 0.1)
rospy.set_param("/move_base/DWAPlannerROS/yaw_goal_tolerance", 0.2)

sound_client = SoundClient()

sm = smach.StateMachine(outcomes=[])

with sm:
    smach.StateMachine.add('FOLLOWING', Following(),
                          transitions={'turn':'TURNING',
                                       'atL2': 'LOCATION2',
                                       'atL4':'LOCATION4'})
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
    smach.StateMachine.add('LOCATION4', Location4(),
                          transitions={'done':'FOLLOWING'})


sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
sis.start()

time.sleep(2)

sm.execute()
