Competition Report 3 - CMPUT 412
================================

##### Video Demo - https://youtu.be/dowRymcstMw
##### Source Code - https://github.com/nwoeanhinnogaehr/412-W19-G5-public/tree/master/comp3

### Purpose

This project can be used to to navigate a turtlebot around the following course, and count/detect
objects at each of the 4 locations.

![ui_v1.0](https://github.com/nwoeanhinnogaehr/412-W19-G5-public/blob/master/media/comp3_course.png?raw=true)

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
Each parking spot may contain one of two items: a billboard with a red shape (like those at location 2 and 3), or an AR tag.
The robot should indicate with the LEDs and a sound if it finds the shape matching the green shape
from location 2, if it finds an AR tag, or if the number of the parking spot matches a specific spot
indicated by the TA.

### Pre-requisites

This project runs on Ubuntu 16.04 with ROS Kenetic installed.

Dependencies: `numpy`, `rospy`, `smach`, `smach_ros`, `cv2`, `cv_bridge`, `tf.transformations`, `imutils`,
`sound_play`

Installation Instructions: Download the package `comp3` from this github repo
and place them in your workspace.

### Execution

Once you have the package in your workspace, you can execute it using the following command:

`$ roslaunch comp3 robot.launch`

It is expected that the robot will start on the line just before the first location.

The following arguments exist when executing the package:

* `use_turtlebot` - true if you are not using simulator, default: true
* `use_velocity_smoother` - true if you want to use velocity_smoother package for the turtlebot, default: true

### Concepts & Code

The main code can be found in [counter.py](https://github.com/nwoeanhinnogaehr/412-W19-G5-public/blob/master/comp3/counter.py).

##### Line Following
Line following behavior is based on the [followbot code](https://github.com/osrf/rosbook/blob/master/followbot/follower_p.py).
Changes made were:

* Integration of a PID controller
* Modification of colour mask to match the competition environment
* Modification of the region mask to match our camera placement (closer to the ground than the default placement)

##### Stopping
Since the course is unchanging, we used a global index to keep track of the current location of the turtlebot.
This way it knows what line it's approaching next in order to take the appropriate action. If it's a line it
needs to stop at it will do so, but if it's a line it needs to turn at it will delay stopping till the turtlebot
is directly over top of the line then it will stop and turn accordingly. Lastly, at location 2 there is no red line.
To stop correctly, the turtlbot uses its laser scanner to check to see if the average distance in front of it is less
then 1.1 metre. If this condition is satisfied, the turtlebot stops and knows it's at location 2.

To detect a red line, the camera looks for a certain amount of red in its line of sight. If this threshold is reached,
the turtlebot asummes it has reached a red line and stops accordingly.

##### Location 1
At location 1, we used two different implementations to detect how many objects were in front. The first one used the data from the laser scanner to run the `get_objects()` function. This function takes the laser scanner data and returns a list of ranges for each object found that was above the size of 60 units wide. The second implementation used the `detect(image, color, cutoff=7000)` function from the `shape_detect` library we made which returned the number of red objects that were found from the camera of the turtlebot. 7000 is the minimum number of pixels in a region for it to be counted. Though we found some inconsistencies in both implementations, we decided to used the laser scanner based detection implementation for the competition. The robot will indicate the number of objects it found by playing a sound and turning on the LEDs.

##### Location 2
At location 2, the shapes are detected using the camera. The robot detects when it should stop by measuring the distance to the billboard with the laser scanner. When the robot reaches the end of the line, it turns slightly to center itself, then based on a single frame from the camera it will separately detect the green and red shapes. The total number of shapes detected will be indicated with a sound and by the LEDs. It stores the detected green shape in a global value so that it can be compared against the shapes at location 3. All shape detection is done using the `shape_detect` module. A cutoff of 1000 is used this time because the shapes are further away. Afterwards it turns around and drives back along the line until it reaches the main line.

##### Location 3

Location 3 has 3 parts to it. The robot stops at each, turning 90 degrees to the left, and detects the red shape using the `shape_detect` module. A cutoff of 7000 is used, because the shapes are close up. If the shape matching the green shape from location 2 is found, the robot will make a "notification" sound. After the robot turns back to the main line, it backs up a bit so that it doesn't miss the next red line, which may have fallen outside the view region.

##### Location 4

At the off ramp, the robot switches from line following to waypoint navigation.
We send an initial pose which is known ahead of time to set up the navigation node.
Because of noise in sensor data, we don't know if the robot has stopped exactly at the start of the off ramp,
but we know it is approximately in that position. To handle this, we set the covariance matrix of the pose
to 0.01 times the identity matrix. Experimentally, this works reasonably well.

We then use waypoint navigation to drive into each parking spot. Once the turtlebot reaches a parking spot, it backs up to observe what the tag is, if any. 
This was necessary because the tag was not always fully visible from within the parking spot.
After noting what tag was there, the robot drives back into the parking spot and indicates the type of tag. The turtlebot makes a cow sound if it detects nothing in the parking spot, a cat sound if it dectects a shape in the parking spot, and a chicken sound if it detects an AR tag in the parking spot.

After the robot has navigated to all parking spots, it navigates to a final waypoint located on the off ramp.
From this point it resumes line following and completes the rest of the course.

### AR tag detection

We use the `ar_track_alvar` node for detecting and estimating the pose of AR tags: https://wiki.ros.org/ar_track_alvar

### Waypoint navigation

Mapping is done using `gmapping`, with
waypoint navigation performed using the `amcl_demo` node with a map of location 4 that we made.
https://wiki.ros.org/gmapping


![ui_v1.0](https://github.com/nwoeanhinnogaehr/412-W19-G5-public/blob/master/media/comp3_map.png?raw=true)

### Shape detect module

Shape detection is based on this tutorial: https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/

To use it, call the `detect` function from `shape_detect.py`. It is essentially a wrapper
around the code from the tutorial, modified to support multiple colours and different
detection thresholds.

### State Diagram

Our package uses Smach to base our robots behavior off a state machine. Run `rosrun smach_viewer smach_viewer.py` while the robot is running to get a live view of the state diagram as shown below

![ui_v1.0](https://github.com/nwoeanhinnogaehr/412-W19-G5-public/blob/master/media/smach.png?raw=true)

### References

https://github.com/osrf/rosbook/blob/master/followbot/follower_p.py

https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/

https://github.com/nwoeanhinnogaehr/412-W19-G5-public/tree/master/comp3
