Baxter Teleoperation system using HTC Vive
ROS Kinetic on Ubuntu 16.04

Dependencies:
Python 2.7  https://www.python.org/downloads/
ROS Kinetic http://wiki.ros.org/kinetic
Baxter Robot or Simulator http://sdk.rethinkrobotics.com/wiki/Workstation_Setup
Steam VR http://store.steampowered.com/steamvr
PyOpenVR https://github.com/cmbruns/pyopenvr

Baxter must be enabled as following instructions in: http://sdk.rethinkrobotics.com/wiki/Workstation_Setup

A virtual Baxter Gazebo simulation can be also be used: http://sdk.rethinkrobotics.com/wiki/Simulator_Installation


To build the teleoperation application:

mkdir -p baxter_ws/src or similar
cd baxter_ws/src
copy contents of htc_publisher, baxter_subscriber & zed packages into workspace
cd ..
# or catkin build
catkin_make
source devel/setup.bash


To run the teleoperation application:

1) Launch SteamVR & switch on all devices
2) Run Room Setup to calibrate HTC Vive

3) Launch HTC publisher:
$ cd ~/baxter_ws
$ ./baxter.sh
$ cd ~/baxter_ws/src/htc_publisher
$ roslaunch htc_publisher htc_publisher.launch

*OPTIONAL IF RUNNING BAXTER SIMULATION*
3.1) Launch RVIZ for tf visualisation:
$ rosrun rviz rviz
3.2) Launch Baxter Simulation: 
$ roslaunch baxter_gazebo baxter_world.launch

4) Launch Baxter Subscriber:
$ cd ~/baxter_ws
$ ./baxter.sh
$ cd ~/baxter_ws/src/baxter_subscriber
$ python main.py

To run the zed_opengl application:

Launch ZED wrapper:
$ cd ~/baxter_ws
$ ./baxter.sh sim
$ cd ~/baxter_ws/src/zed_wrapper
$ roslaunch zed_wrapper display_zedm.launch

Launch ZED subscriber for left eye:
$ cd ~/baxter_ws
$ ./baxter.sh sim
$ cd ~/baxter_ws/src/zed_opengl/src
$ python left_eye.py

Launch ZED subscriber for right eye:
$ cd ~/baxter_ws
$ ./baxter.sh sim
$ cd ~/baxter_ws/src/zed_opengl/src
$ python right_eye.py