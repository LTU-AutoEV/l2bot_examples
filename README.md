# l2bot_examples

Examples for use with LTU's L2Bot



# Installation


## ROS


### Install L2bot

Make sure you have gone through the installation procedure for the L2Bot interface:


  - Installation instructions: [https://github.com/LTU-AutoEV/l2bot](https://github.com/LTU-AutoEV/l2bot)


### Install Examples

Clone this repository into the `src` directory of `l2bot_ws` created during the L2Bot install.

```
$ cd ~/l2bot_ws/src
~/l2bot_ws/src$ git clone https://github.com/LTU-AutoEV/l2bot_examples.git
```


### Install dependencies

**Joystick**
```
~/l2bot_ws$ sudo apt-get install ros-kinetic-joy ros-kinetic-joystick-drivers
```

**Camera**

To use camera a camera with ROS, we must first install opencv.

```
~/l2bot_ws$ sudo apt-get install ros-kinetic-cv-bridge
~/l2bot_ws$ sudo apt-get install ros-kinetic-vision-opencv
```

**rosdep**
Once other dependencies are installed, run the following command

```
~/l2bot_ws$ rosdep install --from-paths src --ignore-src -r -y
```

### Build

Run `catkin_make` from the workspace directory.

```
~/l2bot_ws/src$ cd ..
~/l2bot_ws$ catkin_make
~/l2bot_ws$ source devel/setup.sh
```


# Running the examples

If you get the following error, remember to run `source devel/setup.bash` from the workspace directory.

```
[NODE.launch] is neither a launch file in package [l2bot_examples] nor is [l2bot_examples] a launch file name
The traceback for the exception was written to the log file

```

### `forward`


This example makes the l2bot drive forward for 8 seconds.

```
roslaunch l2bot_examples forward.launch
```

### `joy_nav`

This example allows the user to drive the l2bot with a joystick. This example is currently know to work with a basic Logitech controller.

```
roslaunch l2bot_examples joy_nav.launch
```

### `cam_pub` and `cam_edge_detect`

This example demonstrates how to publish and subscribe to images in ROS.
It also demonstrates simple OpenCV library usage.

To change the input camera, you can change the `source` parameter in `claunch/camera.launch`.

```
roslaunch l2bot_examples camera.launch
``````

### `stop_on_white`

The L2Bot will drive forward until the camera detects a large white object.
This example demonstrates how to control the l2bot based on camera input only.

To change the input camera, you can change the `source` parameter in `claunch/camera.launch`.
You may also need to change the threshold values in `src/stop_on_white.cpp` based on the 
lighting conditions in the environment.
```
roslaunch l2bot_examples stop_on_white.launch
``````
