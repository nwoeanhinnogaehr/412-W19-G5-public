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

Dependencies: `numpy`, `ros_numpy`, `smach`, `smach_ros`

Installation Instructions: Download the packages `follow` and `escape` from this github repo
and place them in your workspace.

### Execution

Once you have the two packages in your workspace, you can execute an instance
of either type by using the commands:

* Cop/Follow - `$ roslaunch follow comp1.launch`
* Robber/Escape - `$ roslaunch escape comp1.launch`

The following argument exist for either instance:

* `use_camera` - true if you are not using simulator, default: true
* `use_turtlebot` - true if you are not using simulator, default: false
* `use_velocity_smoother` - true if you want to use velocity_smoother package for the turtlebot, default: true

### Concepts & Code

The escape instance uses it's laser scan sensors and bumper sensors
to manuver around it's enviroment intellegenctly avoiding obstacles or walls it can
see in its range. The follow instance uses data from the laserscan/pointcloud to
detect any near objects, and move toward the object with the highest score evaluated
by:

1. the objects coordinates in relation to the previously tracked object
2. the size of the object
3. the average color of the object
