# Baxter Teleoperation system using HTC Vive

A Baxter teleoperation system that runs on a single Linux Ubuntu (16.04) machine.  An example of this system being demonstrated is available <a href="https://www.youtube.com/watch?v=3GzbZMFeiFI">here</a>

## Getting Started
Follow these instructions to get a copy of the project up and running on your local machine for development and testing purposes.

### Dependencies

```
Python 2.7  https://www.python.org/downloads/
ROS Kinetic http://wiki.ros.org/kinetic
Baxter Robot or Simulator http://sdk.rethinkrobotics.com/wiki/Workstation_Setup
Steam VR http://store.steampowered.com/steamvr
PyOpenVR https://github.com/cmbruns/pyopenvr

Baxter must be enabled as following instructions in: http://sdk.rethinkrobotics.com/wiki/Workstation_Setup
A virtual Baxter Gazebo simulation can be also be used: http://sdk.rethinkrobotics.com/wiki/Simulator_Installation
```

### Installation

To build the teleoperation system, create a ROS workspace

```bash
mkdir -p baxter_ws/src or similar
cd baxter_ws/src
git clone https://github.com/kwunh/vive_teleoperation
cd ..
# or catkin build
catkin_make
source devel/setup.bash
```

## Running applications:

### Running teleoperation system

1) Launch SteamVR & switch on all devices
2) Run Room Setup to calibrate HTC Vive

3) Launch HTC publisher:
```bash
$ cd ~/baxter_ws
$ ./baxter.sh
$ cd ~/baxter_ws/src/htc_publisher
$ roslaunch htc_publisher htc_publisher.launch
```

*OPTIONAL IF RUNNING BAXTER SIMULATION*

3.1) Launch RVIZ for tf visualisation:
```bash
$ rosrun rviz rviz
```
3.2) Launch Baxter Simulation:
```bash
$ roslaunch baxter_gazebo baxter_world.launch
```

4) Launch Baxter Subscriber:
```bash
$ cd ~/baxter_ws
$ ./baxter.sh
$ cd ~/baxter_ws/src/baxter_subscriber
$ python main.py
```

Published topics:
```
Topic                     Type                        Rate
/tf                       tf2_msgs/TFMessage          250Hz
/vive_left                sensor_msgs/Joy             On Event
/vive_right               sensor_msgs/Joy             On Event
/left_controller_as_ps    geometry_msgs/PoseStamped   30Hz
/right_controller_as_ps   geometry_msgs/PoseStamped   30Hz
```

### Running zed_opengl

1) Launch ZED wrapper
```bash
$ cd ~/baxter_ws
$ ./baxter.sh sim
$ cd ~/baxter_ws/src/zed_wrapper
$ roslaunch zed_wrapper display_zedm.launch
```

2) Launch ZED subscriber for left eye:
```bash
$ cd ~/baxter_ws
$ ./baxter.sh sim
$ cd ~/baxter_ws/src/zed_opengl/src
$ python left_eye.py
```

3) Launch ZED subscriber for right eye:
```bash
$ cd ~/baxter_ws
$ ./baxter.sh sim
$ cd ~/baxter_ws/src/zed_opengl/src
$ python right_eye.py
```

### Running zed_htc (Unity Application)

Ensure ZED Camera and HTC Vive are connected to the Linux machine.  

Simply double click the provided executable in zed_htc: zed_camera.x86_64

