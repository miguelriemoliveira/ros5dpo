# rtdb2ros
h1. About the Project

This project is designed to provide an interface between the RTDB and ROS.

The database is Real-Time Database (RTDB) middleware for collaborative robotics. Check https://code.google.com/p/rtdb/ for more info.

The need for interfacing ROS with the RTDB is to be able to use the CAMBADA's simulation tool. Check http://robotica.ua.pt/CAMBADA/. 

h1. Prerequisites

We need the CAMBADA public code. To install run:

 git clone https://github.com/CAMBADA/cambada-pub  

Follow the instructions on the README file

NOTE: although they say its for Ubuntu 14.04, I have tested with 12.04 LTS and it works fine.

h1. Installation

Download the rtdb2ros package into one of your catkin workspaces (or create a new one)

 git clone https://github.com/miguelriemoliveira/rtdb2ros

To compile, from your catkin_ws directory, run

 catkin_make

h1. How to run

First, we need to launch the CAMBADA simulator, from the cambada-pub/bin directory, run the following commands.
To launch the simulator and basestation

 sudo ./sim

To launch a CAMBADA agent (e.g., we will use agent number 5)
 
 sudo ./simAgents 5

To run a rtdb2ros node for agent 3, first go to the devel/lib/rtdb2ros directory

 cd ~/catkin_ws/devel/lib/rtdb2ros

Set the environment environment variable AGENT to 3

 export AGENT=3

Run the node

 sudo -E ./rtdb2ros_node


h1. Subscribed topics

The node subscribes to topic 


> /velocity_cmd_agent[AGENT]([http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html | geometry_msgs/Twist])

where AGENT is the environment variable we set before.

Tip: you can set the default agent to 1 by adding export AGENT=1 to your .bashrc

h1. Published topics

The node publishes

> /odom_agent[AGENT]([http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html | nav_msgs/Odometry])

In addition, also a tf transform is also broadcasted, from /odom_agent[AGENT] to /base_footprint_agent[AGENT]

h1. How to test

To test, after launching a rtdb node (for example for AGENT=3), we need to give a velocity command

 rostopic pub /velocity_cmd_agent3 geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 90.0" --rate 10

The robot 3 in the simulator window starts moving around, which proves the rtdb2ros_node is correctly writing to the rtdb. 
To test the reading from the rtdb functionality, we can visualize the broadcasted transforms using rviz

 rosrun rviz rviz

Add a new TF marker, and set Fixed Frame to /odom_agent3

You should see the robot moving around in rviz
