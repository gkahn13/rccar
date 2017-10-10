# Installation Instructions
#### Setup ROS

Add to .bashrc
```bash
source /opt/ros/indigo/setup.bash
```

Then create your ROS workspace
```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws/
$ catkin_make
```

Add to .bashrc
```bash
source ~/catkin_ws/devel/setup.bash
```

#### Setup ZED ROS

```bash
$ sudo apt-get install libpcl-1.7-all ros-indigo-pcl-ros ros-indigo-image-view
$ git clone https://github.com/stereolabs/zed-ros-wrapper.git
$ cd ~/catkin_ws
$ catkin_make
```

Test that it works
```bash
$ roslaunch zed_wrapper zed.launch
$ rosrun image_view image_view image:=/camera/right/image_rect_color
```

#### Setup this repo

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/rctn/rccar.git
$ roslaunch rccar rccar.launch use_zed:=true
$ rosrun image_view image_view image:=/rccar/zed/right/image_rect_color
```

And rostopic echo all the topics to check

# Running experiments

Go into rccar.launch and change the bagpath to where you want to record data to (e.g. flash drive)

```bash
$ roslaunch rccar rccar.launch use_zed:=true record:=true use_pid:=false
```

To check that it is working:
- rostopic echo some topics
- view the image streams using image_view (see "Setup this repo" for the ROS command syntax)
- if using the Corsair flash drive, check that the LED flashes every ~30 seconds (this means data is being copied to it from the TX1)

#### Velocity control

If you want to run the car with a constant velocity PID controller, run roslaunch with use_pid:=true

What this does is if you are in human control mode and switch to caffe mode, the constant velocity controller will maintain the last encoder value it saw when in human control mode.

The example use case is to calibrate, launch the script, go into human control mode and drive at the desired speed, switch into caffe mode, and then all you have to do is steer.
