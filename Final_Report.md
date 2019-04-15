Final Report - CMPUT 412
================================

##### Video Demo - https://youtu.be/f02AYBRuTwo
##### Source Code - [comp4](https://github.com/nwoeanhinnogaehr/412-W19-G5-public/tree/master/comp4), [comp5](https://github.com/nwoeanhinnogaehr/412-W19-G5-public/tree/master/comp5)

### Purpose

This project can be used to to navigate a turtlebot around the following course, and count/detect/manipulate
objects at each of the 4 locations, according to a set of predefined rules.

![ui_v1.0](https://github.com/nwoeanhinnogaehr/412-W19-G5-public/blob/master/media/comp4_course.png?raw=true)

At the first location, the turtlebot turns to the left and detects the number of objects that are in
front of it. This will be a number between 1 and 3, and the turtlebot will make a sound of either "one",
"two", or "three" depending on its calculations. The two LED's on the turtlebot will also represent in
binary the number of objects that were detected. The following is an example of location 1 with 3
objects.

![ui_v1.0](https://github.com/nwoeanhinnogaehr/412-W19-G5-public/blob/master/media/loc1.png?raw=true)

At location 2, the turtlebot will complete two tasks. The first task is to count the objects once again,
and indicate "one", "two", or "three" along with a binary LED representation of how many objects were
detected. The second task is to recognize the one green object that is present at the location, and then
detect the shape of that object. The following is an example of location 2 with 3 objects.

![ui_v1.0](https://github.com/nwoeanhinnogaehr/412-W19-G5-public/blob/master/media/loc2.png?raw=true)

At location 3, there are 3 differnt places for the turtlebot to stop. At each stop, there is a single
shape, and the turtlebot makes an alert noise if the shape matches the green shape that was detected
at location 2. The following is an example of a single stop at location 3.

![ui_v1.0](https://github.com/nwoeanhinnogaehr/412-W19-G5-public/blob/master/media/loc3.png?raw=true)

Location 4, the "parking lot", consists of 8 parking spots marked by red tape, as shown in the image at the top.

Parking spots 6, 7 and 8 contain a billboard with a red shape (like those at location 2 and 3). When the robot
finds the stall with the red shape matching the green shape from location 2, it will park in the stall and
make a sound.

Parking spots 1-5 are used for the box pushing exercise. One of the spots will have an AR tag placed behind
the parking spot indicating a goal location. A box covered in AR tags will be placed in one of the remaining
spots (1-5). The robot checks each spot for the appropriate AR tags, indicating with sound and LED when it finds both tags.
Next, it must push the box to the goal spot, and indicate again with the LED and a sound when it has done this.

After completing these two tasks, the robot returns back to the line and continues on to location 3 (yes,
location 4 comes before location 3). The following is what location 4 looks like.

![ui_v1.0](https://github.com/nwoeanhinnogaehr/412-W19-G5-public/blob/master/media/loc_2.png?raw=true)

### Pre-requisites

This project runs on Ubuntu 16.04 with ROS Kenetic installed.

Dependencies: `numpy`, `rospy`, `smach`, `smach_ros`, `cv2`, `cv_bridge`, `tf.transformations`, `imutils`,
`sound_play`, `usb_cam`, `ar_track_alvar`

Installation Instructions: Download the package `comp4` or `comp5` from this github repo
and place it in your ROS kinetic workspace.

### Turtlebot Configuration

In addition to the ASUS camera, you will need a second external camera. A foam block on the front
of the robot is also needed for box pushing.

See the image below for an example of the competition 4 configuration. For competition 5,
the USB camera should be in the same spot, but the asus camera is placed on the right side instead.

In both cases, the logitech camera should be centered and facing down at the steepest possible angle
such that the foam block remains out of view. 

![ui_v1.0](https://github.com/nwoeanhinnogaehr/412-W19-G5-public/blob/master/media/IMG_20190409_155607.jpg?raw=true)

### Execution

Once you have the package in your workspace, you can execute it using the following command:

`$ roslaunch comp4 robot.launch`

It is expected that the robot will start on the line just before the first location.

The following arguments exist when executing the package:

* `usb_camera_path` - path to usb camera device default: `/dev/video1`

### Concepts & Code

The main code can be found in the counter.py file in the appropriate package.

Lots of the code is identical between the two competitions, but in competition 5, we removed most of the
functionality besides box pushing, and increased the speed of line following.
The box pushing code was also rewritten in comp5 to improve speed. However, the general approach remains
the same.

##### Line Following
Line following behavior is based on the [followbot code](https://github.com/osrf/rosbook/blob/master/followbot/follower_p.py).
The USB camera is used for line following.
Changes made were:

* Integration of a PID controller.
* Modification of colour mask to match the competition environment. We convert the image to HSV first, then
threshold it using `cv2.inRange`.
* Modification of the region mask to match our camera placement (closer to the ground than the default placement).

##### Stopping
Since the course is unchanging, we used a global index to keep track of the current location of the turtlebot.
This way it knows what red line it's approaching next in order to take the appropriate action. If it's a line it
needs to stop at it will do so, but if it's a line it needs to turn at it will delay stopping till the turtlebot
is directly over top of the line then it will stop and turn accordingly. Lastly, at location 2 there is no red line.
To stop correctly, the turtlbot uses its laser scanner to check to see if the average distance in front of it is less
then 1.1 metre. If this condition is satisfied, the turtlebot stops and knows it's at location 2.

To detect a red line, we test the USB camera image for a certain amount of red in its line of sight. This is done with
HSV images and the `cv2.inRange` and `cv2.moments` functions, similar to how the white lines are detected. If this
threshold is reached, the turtlebot assumes it has reached a red line and stops accordingly.

##### Location 1
At location 1, we used two different implementations to detect how many objects were in front. The first one used the data from the laser scanner to run the `get_objects()` function. This function takes the laser scanner data and returns a list of ranges for each object found that was above the size of 60 units wide. The second implementation used the `detect(image, color, cutoff=7000)` function from the `shape_detect` library we made which returned the number of red objects that were found from the camera of the turtlebot. 7000 is the minimum number of pixels in a region for it to be counted. Though we found some inconsistencies in both implementations, we decided to used the laser scanner based detection implementation for the competition. The robot will indicate the number of objects it found by playing a sound and turning on the LEDs.

##### Location 2
At location 2, the shapes are detected using the camera. The robot detects when it should stop by measuring the distance to the billboard with the laser scanner. When the robot reaches the end of the line, it turns slightly to center itself, then based on a single frame from the camera it will separately detect the green and red shapes. The total number of shapes detected will be indicated with a sound and by the LEDs. It stores the detected green shape in a global value so that it can be compared against the shapes at location 3. All shape detection is done using the `shape_detect` module. A cutoff of 1000 is used this time because the shapes are further away. Afterwards it turns around and drives back along the line until it reaches the main line.

##### Location 3

Location 3 has 3 parts to it. The robot stops at each, turning 90 degrees to the left, and detects the red shape using the `shape_detect` module. A cutoff of 7000 is used, because the shapes are close up. If the shape matching the green shape from location 2 is found, the robot will make a "notification" sound. Since the red lines are so close together here, we ran into issues with the lines not being visible. To deal with this we simply hard-coded all movement at this location.

##### Location 4

At the off ramp, the robot switches from line following to movement that is largely hard-coded, using the odometer to judge distance.
Since odometer error can accumulate significantly over time, we use the red lines of tape on the ground to periodically realign the robot.

First, the robot examines the shapes placed in location 8, 7 and 6 (in that order). This is done by using preprogrammed movements based on
distances and angles between parking spots. At each parking spot, the robot aligns itself to the red line on the ground. From that position,
it does shape detection to determine which shape is placed in the parking spot. If the shape matches the shape from location 2, then the
robot makes a sound and turns on the LED. For more details on turning, driving, alignment and shape detection, see the respective sections
below.

Next, the robot does the box pushing exercise. First, it scans each of the parking spots 1-5 looking for AR tags. The two tags that it expects
to see for the goal and the box are specified in the code. See below for details on AR tag detection. After it has located both tags,
the robot follows one of two fixed movement patterns depending on which direction it has to push the box. We found that each box is approximately
0.73 meters apart, so we drive by that distance for each square that we need to push the box for.

Finally, the robot leaves the location 4 area and returns to line following towards location 3.

### Turning

Hard coded turns movements are done using a function `drive_for_dist` which integrates yaw messages
from the odometer at a fixed rate. This is significantly more accurate than turning for a specific time.
It also avoids a problem where turns which land directly on the boundary between -PI and PI radians
may be inaccurate due to the angle wrapping around. Without integration, a more complex method for
checking when to stop turning is needed.

### Driving

Hard coded forwards and backwards movements are done using a function `drive_for_dist` which integrates position messages
from the odometer at a fixed rate. This is significantly more accurate than driving for a specific time.

### Alignment

To correct the robot's position in the case that error has accumulated, we wrote a function that aligns the robot perpendicularly
to a red line on the ground. This is done in a similar manner to how line following works, except we take the red image moments
from the left side and the right side of the screen and compare them. The difference between these two moments determines how much
to turn. When red is no longer visible in the bottom part of the screen, the robot stops. When it stops, it will be perpendicular
to the line and the foam block on the front of the robot will be aligned with the closer side of the line.

### AR tag detection

We use the `ar_track_alvar` node for detecting and estimating the pose of AR tags: https://wiki.ros.org/ar_track_alvar

### Shape detect module

Shape detection is based on this tutorial: https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/

To use it, call the `detect` function from `shape_detect.py`. It is essentially a wrapper
around the code from the tutorial, modified to support multiple colours and different
detection thresholds.

### State Diagram

Our package uses Smach to base our robots behavior off a state machine. Run `rosrun smach_viewer smach_viewer.py` while the robot is running to get a live view of the state diagram as shown below

![ui_v1.0](https://github.com/nwoeanhinnogaehr/412-W19-G5-public/blob/master/media/state_diagram.png?raw=true)

### References

https://github.com/osrf/rosbook/blob/master/followbot/follower_p.py

https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/
