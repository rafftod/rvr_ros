# RVR Foraging

This repository contains everything needed to use ARGoS and ROS to run experiments with the Sphero RVR, with additional sensors ([Terabee Teraranger Multiflex](https://www.terabee.com/shop/lidar-tof-multi-directional-arrays/teraranger-multiflex/) and [YDLIDAR X4](https://www.ydlidar.com/products/view/5.html)).

It also includes a basic foraging behaviour.

This environment has been tested with the following environment :

-   Ubuntu 20.04
-   ROS Noetic
-   ARGoS 3 beta 48

## Installation

### Dependencies

Here are the dependencies of the project :

-   [ARGoS 3 beta 48](https://github.com/ilpincy/argos3/tree/3.0.0-beta48)
-   [ROS Noetic for Ubuntu 20.04](http://wiki.ros.org/noetic/Installation/Ubuntu)
-   [RVR plugin for ARGoS 3](https://github.com/rafftod/argos3-rvr)

Other ROS packages dependencies will be installed during the next step.

### Compilation

First clone the repository :

```
git clone https://github.com/rafftod/rvr_foraging rvr_foraging
cd rvr_foraging
git submodule init
git submodule update
```

Then install ROS packages dependencies :

```
rosdep install --from-paths src --ignore-src -r -y
```

Finally compile the workspace packages :

```
catkin_make
```

Source the devel folder to be able to run the package :

```
source ./devel/setup.bash
```

Optional : add it to .bashrc to avoid doing it at each login

```
echo "source ${PWD}/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Running the code

First start a master ROS :

```
roscore
```

You can run each of the following node by entering the corresponding command :

#### RVR driver

```
rosrun rvr_foraging rvr_driver.py
```

#### ARGoS controller

In the workspace directory :

```
argos3 -c rvr.argos
```

#### Teraranger Multiflex

```
rosrun teraranger_array teraranger_multiflex _portname:=/dev/ttyACM0
```

#### YDLIDAR X4

```
roslaunch ydlidar_ros X4.launch
```
