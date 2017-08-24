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
~/l2bot_ws/src$ git clone https://github.com/LTU-AutoEV/l2bot.git
```

### Build

Run `catkin_make` from the workspace directory.

```
~/l2bot_ws/src$ cd ..
~/l2bot_ws$ catkin_make
```

Install dependencies

```
~/l2bot_ws$ sudo apt-get install ros-kinetic-joy ros-kinetic-joystick-drivers
~/l2bot_ws$ rosdep install l2bot_examples
```
