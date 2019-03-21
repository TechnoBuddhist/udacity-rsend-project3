# Project 3 - BasketBot
My project for the Udacity Robotics Software Engineer Nanodegree 2019.

## Getting Started
You need [ROS](http://ros.org). If you aren't familiar with [ROS](http://ros.org) then this project is probably no good to you whatsoever!

I have used ROS Kinetic as this is used on the nanodegree.

I'm not going to talk you through how to use ROS, there are plenty of tutorials online and the [ROS Wiki](http://wiki.ros.org) is particularly useful. You will need a ROS workspace, either one you have currently or [create a new workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

You will also need [RVIZ](http://wiki.ros.org/rviz/UserGuide) and [Gazebo](http://gazebosim.org/).


### Prerequisites
- An installed ROS system with rviz and gazebo
- A ROS workspace

### Installing
You should [create a new package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage).

`catkin_create_pkg <package_name> roscpp std_msgs message_generation`

Copy/clone this project into the workspace/src/<package_name> directory.

## Usage
You will need 2 terminals running.

**Terminal 1**
In your ROS workspace :-

```
catkin_make
source devel/setup.bash
roslaunch basketbot world.launch
```

**Terminal 2**
In your ROS workspace :-

```
source devel/setup.bash
roslaunch basketbot amcl.launch
```

## Future Work
I have started to work on a detector node that should stop the robot when an object is close to it. The code is in this repo but remains non-working currently.

## Contributing
No contributions accepted this is **my project** for the course.

## License
GNU GPLv3

## Attributions & Acknowlegements
