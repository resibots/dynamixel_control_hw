# Hardware interface for Dynamixels and ROS control

[ROS control][] is a framework to design software control loops in [ROS][] (Robot Operating System) where the controller code is decoupled from the actual hardware.

This piece of software provides a hardware interface for ROS control. it's aim is to allow generic software controllers to control a set of Dynamixel actuators.

We are using it for our robots and developing it for this purpose. Version 0.0.1 is clearly no final code but it works for the basic usage.

## Features

- simple, ROS-style control interface for your dynamixel-based robot
- not specific to a given number or set of actuators
- uses radians uniformly over all Dynamixel models (no need to worry about ticks-to-angle conversion)
- you can set an offset for each actuator's position
- set names for your dynamixels in a configuration file

## Limitations
We are currently working on the following features:

- joint limits not implemented yet
- position control only (doesn't manage wheel mode)
- it would be great to offer a service to reset one actuator after an overload error

## Installation and usage
`dynamixel_control_hw` depends on the [libdynamixel][] library. Please install it, it's light and quick.

Then, you'll need to store the installation path of libdynamixel in the `RESIBOTS_DIR` environment variable.

Install the ros package ros_control. In apt-get, it is called `ros-YourDistro-ros-control` (where *YourDistro* must be changed by the ROS distribution's name, like indigo or jade).

Clone the `dynamixel_control_hw` repository in your catkin workspace's source directory and run a `catkin_make`.

### Testing the hardware interface
If you want to use the sample launch files or to use one of the default controllers, please install with apt-get:

- `ros-YourDistro-ros-controllers` and
- `ros-YourDistro-joint-state-publisher`.

Have a look at the `launch/sample.launch` file. It will by default launch two feed-forward only position controllers and a virtual controller that publishes the states of each actuator.

*Before starting* it, check `config/sample.yaml` for the `hardware_mapping` section, the `serial_interface` and `baudrate` settings. Once you are sure that it's correct, you can launch the sample. By looking at the available topics, you should find two for the commands of each joint and one for the joint state.

## Alternative software
If you know of an other software offering similar functionalities to this one, feel free to open an issue so that we can add it here.

## Support and contact
This software is developed as part of the ResiBots project. We do our best to keep it free of bug and to implement relevant features. Should you face an issue or have a suggestion, please open an issue.

## Projects using this software
- [ResiBots][]

[ResiBots]: http://www.resibots.eu
[libdynamixel]: http://github.com/resibots/libdynamixel
[ROS]: http://www.ros.org/
[ROS control]: http://wiki.ros.org/ros_control
