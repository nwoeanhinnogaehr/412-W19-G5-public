Competition Report 1 - CMPUT 412
================================

### Purpose

This project can be used to create either an instance of a turtlebot that tries to
escape an envading turtlebot, or one that tries to follow another turtlebot. For the
follower instance, the other turtlebot must be placed one metre in front of it prior
its execution. Once the follower is running, it works to maintain a safe distance
away from the object it is following (1.2 metres) and moves forward and backs up
accordingly. The escape instance uses motion control to not only avoid obstacles,
but to attempt to manuver around them without having to stop and turn. In the case
that a bumper sensor is hit on the escape instance, it turns around and continues to
envade.

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
