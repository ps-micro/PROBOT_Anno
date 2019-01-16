# PROBOT

This repository provides ROS support for PROBOT.   
This repo holds source code for ROS versions Kinetic.

__Operating System Install__  
Operating system version is not less than Ubuntu linux 14.04, both supports 32bit and 64bit system.
Ubuntu Linux download:http://www.ubuntu.com/download/

__ROS Install__  
Choose any of the following ways to install:   
1.Install ROS with [this tutorials](http://wiki.ros.org/kinetic/Installation/Ubuntu)   
2.Install ROS with PROBOT [install.sh](https://github.com/ps-micro/probot_ros/install.sh) (Option 0)  

__PROBOT Dependant Packages Install__  
Choose any of the following ways to install:   
1.Install commands:   

    $ ROS_VERSION=`/usr/bin/rosversion -d`
    $ sudo apt-get install ros-${ROS_VERSION}-moveit-*
    $ sudo apt-get install ros-${ROS_VERSION}-industrial-*
    $ sudo apt-get install ros-${ROS_VERSION}-gazebo-ros-control
    $ sudo apt-get install ros-${ROS_VERSION}-ros-control ros-${ROS_VERSION}-ros-controllers
    $ sudo apt-get install ros-${ROS_VERSION}-trac-ik-kinematics-plugin
    $ sudo apt-get install ros-${ROS_VERSION}-usb-cam

2.Install with PROBOT [install.sh](https://github.com/ps-micro/probot_ros/install.sh) (Option 1)  

__PROBOT Packages Install__  
Choose any of the following ways to install:   
1.Install commands:   
- Set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).
- Clone the repository into the src/ folder  
- Use "catkin_make" to build workspace
- Use "catkin_make install" to install librarys and packages
- Set up workspace environment:   
```echo "source ~/WORKSPACE_PATH/install/setup.bash" >> ~/.bashrc```

2.Install with PROBOT [install.sh](https://github.com/ps-micro/probot_ros/install.sh) (Option 2) 

__MoveIt! with real Hardware__  
There are launch files available to bringup a real robot.   

Don't forget to source the correct setup shell files and use a new terminal for each command! 

To bring up the real robot, then run:

```roslaunch roslaunch probot_bringup probot_anno_bringup.launch robot_ip:=192.168.2.123```

You can use MoveIt! plugin to control the robot.

Additionally, a simple test script that moves the robot to predefined positions can be executed like this:

```rosrun probot_demo test_move.py```

CAUTION:  
Remember that you should always have your hands on the big red button in case there is something in the way or anything unexpected happens.

__MoveIt! with a simulated robot__  
Again, you can use MoveIt! to control the simulated robot. 

For setting up the MoveIt! to allow motion planning run:
```roslaunch probot_bringup probot_anno_bringup.launch sim:=true```

__MoveIt! with Gazebo Simulation__  
There are launch files available to bringup a simulated robot.

To bring up the simulated robot in Gazebo and moveit, run:
```roslaunch probot_gazebo probot_bringup_moveit.launch```
