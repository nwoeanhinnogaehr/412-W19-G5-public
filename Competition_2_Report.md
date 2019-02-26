Competition Report 2 - CMPUT 412
================================

### Purpose

This project can be used to to navigate a turtlebot around the following course, and count/detect 
objects at each of the 3 locations.

![ui_v1.0](https://github.com/nwoeanhinnogaehr/412-W19-G5-public/blob/master/media/course.png?raw=true)
![ui_v1.0](https://github.com/nwoeanhinnogaehr/412-W19-G5-public/blob/master/media/course_irl.png?raw=true)

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

### Pre-requisites

This project runs on Ubuntu 16.04 with ROS Kenetic installed.

Dependencies: `numpy`, `rospy`, `smach`, `smach_ros`, `cv2`, `cv_bridge`, `tf.transformations`, `imutils`,
`subprocess`

Installation Instructions: Download the package `comp2` from this github repo
and place them in your workspace.

### Execution

Once you have the package in your workspace, you can execute it using the following command:

`$ roslaunch comp2 course.launch`

### Concepts & Code

##### Line Following

##### Stopping
Since the course is unchanging, we used a global index to keep track of the current location of the turtlebot. 
This way it knows what line it's apporaching next in order to take the appropriate action. If it's a line it
needs to stop at it will do so, but if it's a line it needs to turn at it will delay stopping till the turtlebot
is directly overtop of the line then it will stop and turn accordingly. Lastly, at location 2 there is no red line.
To stop correctly, the turtlbot uses its laserscanner to check to see if the average distance in front of it is less 
then 1.1 metre. If this condition is satisfied, the turtlebot stops and knows it's at location 2.<br/>

To detect a red line, the camera looks for a certain amount of red in its line of sight. If this threshold is reached,
the turtlebot asummes it has reached a red line and stops accordingly.

##### Location 1
At location 1, we used two differnt implementations to detect how many objects were in front. The first one used the data from the laser scanner to run the get_objects() function. This function takes the laserscanner data and returns a list of ranges for each object found that was above the size of 60 units wide. The second implementation used the detect(image, color, cutoff=7000) function from the shape_detect library we made which returned the number of red objects that were found from the camera of the turtlebot. Though we found some inconsistances in both implementations, we decided to used the color detection implemtation for the competition. 
##### Location 2

##### Location 3
