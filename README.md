# Hardware interface for Dynamixels and ROS control

[ROS control](http://wiki.ros.org/ros_control) is a framework to design software control loops in [ROS](http://www.ros.org/) (Robot Operating System) where the controller code is decoupled from the actual hardware.

This piece of software provides a hardware interface for ROS control. it's aim is to allow generic software controllers to control a set of Dynamixel actuators.

We are using it for our robots and develosping it for this purpose. Version 0.0.1 is clearly no final code but it works for the basic usage.

## Features

- simple, ROS-style control interface for your dynamixel-based robot
- not specific to a given number or set of actuators
- set names for your dynamixels in a configuration file

## Limitations

- doesn't manage wheel mode
- joint limits not implemented yet
- assumes 4096 steps for 360°
- position control only
- misses an offset table
- allow reset of the actuators in case of overload error

## Installation and usage
You are assumed to work in a POSIX environment.

`dynmaixel_control_hw` depends on the [libdynamixel](https://github.com/resibots/libdynamixel) library. Once it is installed, you'll need to put the path of the installed library in the `RESIBOTS_DIR` environment variable.

Install the ros package ros_control. In apt-get, it is called `ros-YourDistro-ros-control` (where *YourDistro* must be changed by the ROS distribution's name, like indigo or jade). If you plan to the sample launch files or to use one of the default controllers, please also install `ros-YourDistro-ros-control` and `ros-YourDistro-joint-state-publisher` with apt-get.

Clone the `dynmaixel_control_hw` repository in your catkin workspace's source directory and run a `catkin_make`.

## Alternative software
If you know of an other software offering similar functionalities to this one, feel free to open an issue so that we can add it here.

## Support and contact
This software is developed as part of the ResiBots project. We do our best to keep it free of bug and to implement relevant features. Should you face an issue or have a suggestion, please open an issue.

## Projects using this software
- [ResiBots](http://www.resibots.eu)


[libdynamixel]: http://github.com/resibots/libdynamixel
