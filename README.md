# RVR Foraging

This repository contains everything needed to use ARGoS and ROS to run experiments with the Sphero RVR, with additional sensors ([Terabee Teraranger Multiflex](https://www.terabee.com/shop/lidar-tof-multi-directional-arrays/teraranger-multiflex/) and [YDLIDAR X4](https://www.ydlidar.com/products/view/5.html)).

It also includes a basic foraging behaviour.

## Testing environment

This environment has been tested with the following environment :

-   Ubuntu 20.04
-   ROS Noetic
-   ARGoS 3 beta 48

## Current implementation

### Content

This implementation currently supports :

-   Wheels actuators
-   Ground color sensor
-   RGB LEDs
-   Proximity sensors
-   LIDAR

### Architecture

The architecture is as follows :
![RVR ROS mode of functioning](https://i.imgur.com/2DQQTra.png "RVR ROS")

It revolves around 4 main nodes :

-   The RVR driver node that enables the use of the real robot sensors and actuators
-   The ARGoS node that centralizes the control software
-   The LIDAR node
-   The proximity sensor nodes

These are structured as follows :

```
rvr_foraging/
├─ src/
│  ├─ rvr_foraging/
│  │  ├─ src/
│  │  │  ├─ foraging.h
│  │  │  ├─ foraging.cpp
│  │  │  ├─ rvr_driver.py
│  ├─ serial/
│  ├─ teraranger_array/
│  ├─ ydlidar_ros/
rvr.argos
sim.argos
```

The inner rvr_foraging folder is the package I developed. The other ones in src/ are dependencies for the external sensors and belong to other repositories.

#### foraging.h and foraging.cpp

These include the implementation of the ARGoS controller and its ROS node.

#### rvr_driver.py

This is the bridge between the ARGoS controller and the RVR API.

Note: exiting this script with Ctrl+C does not always work and sometimes requires a reboot of the robot.

#### rvr.argos

This starts the controller with the `rvr_driven` parameter set as `true`, meant to be run on the real robot.

#### sim.argos

This starts the controller with the `rvr_driven` parameter set as `false`, meant to be run on simulation.

## Installation

### Dependencies

Here are the dependencies of the project :

-   [ARGoS 3 beta 48](https://github.com/ilpincy/argos3/tree/3.0.0-beta48)
-   [ROS Noetic for Ubuntu 20.04](http://wiki.ros.org/noetic/Installation/Ubuntu)
-   [RVR plugin for ARGoS 3](https://github.com/rafftod/argos3-rvr)

You will also need the Sphero SDK for Raspberry Pi.

Install its dependencies :

```
pip3 install aiohttp pyserial_asyncio
```

Clone the SDK :

```
cd ~
git clone https://github.com/sphero-inc/sphero-sdk-raspberrypi-python
```

Check that UART and the API is working :

```
python3 ~/sphero-sdk-raspberrypi-python/getting_started/observer/api_and_shell/echo.py
```

You should get 2 lines of output after the RVR firmware check. If you don't, check that UART is correctly set up on the Raspberry Pi.

### Compilation

First clone the repository :

```
cd ~
git clone https://github.com/rafftod/rvr_foraging rvr_foraging
cd rvr_foraging
git submodule init
git submodule update
```

Create a symbolic link to the Sphero SDK

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

Create a symbolic link to the SDK in the workspace :

```
ln -s ~/sphero-sdk-raspberrypi-python/sphero_sdk/ ~/rvr_foraging/src/rvr_foraging/src/
```

Rename the LIDAR port to /dev/ydlidar

```
roscd ydlidar_ros/startup
sudo chmod 777 ./*
sudo sh initenv.sh
```

## Running the code

First start a master ROS :

```
roscore
```

You can run each of the following node by entering the corresponding command :

### RVR driver

```
rosrun rvr_foraging rvr_driver.py
```

### ARGoS controller

In the workspace directory :

Start a simulation :

```
argos3 -c sim.argos
```

Start the real robot controller :

```
argos3 -c rvr.argos
```

### Teraranger Multiflex

```
rosrun teraranger_array teraranger_multiflex _portname:=/dev/ttyACM0
```

### YDLIDAR X4

```
roslaunch ydlidar_ros X4.launch
```
