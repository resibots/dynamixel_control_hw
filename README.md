# Hardware interface for Dynamixels and ROS control

[ROS control][] is a framework to design software control loops in [ROS][] (Robot Operating System) where the controller code is decoupled from the actual hardware.

This piece of software provides a hardware interface for ROS control. Its aim is to allow generic software controllers to control a set of Dynamixel actuators.

We are using it for our robots and developing it for this purpose. Version 0.1.0 is clearly no final code but it works for the regular needs.

## Features

- simple, ROS-style control interface for your dynamixel-based robot
- not specific to a given number or set of actuators
- position and velocity control
- works with both version of Dynamixel protocol (1 and 2)
- uses radians uniformly over all Dynamixel models (no need to worry about ticks-to-angle conversion)
- you can set an offset for each actuator's position
- set names for your dynamixels in a configuration file

## Limitations
We are currently working on the following features:

- joint limits not implemented yet
- add support for the more exotic control modes (including current and multi-turn)
- it would be great to offer a service to reset one actuator after an overload error

## Installation and usage
`dynamixel_control_hw` depends on the [libdynamixel][] library. Please install it, it's light and quick.

Then, you'll need to store the installation path of libdynamixel in the `LIBDYNAMIXEL` environment variable.

Install the ros package ros_control. In apt-get, it is called `ros-YourDistro-ros-control` (where *YourDistro* must be changed by the ROS distribution's name, like indigo or jade).

Clone the `dynamixel_control_hw` repository in your catkin workspace's source directory and run a `catkin_make`.

### Parameters
For examples cofiguration files, see the ones in `config/`. Here are the accepted parameters.

- **serial_interface**: path to the USB to serial interface for example "/dev/ttyUSB0"
- **baudrate**: baud-rate for the serial communication with actuators (in bauds)
- **loop_frequency**: frequency at which the control loop will run (in Hz)
- **cycle_time_error_threshold**: how much delay is tolerated on the control loop (in s)
- **read_timeout**: timeout on the reception of replies from the servos (in s)
- **read_timeout**: (for the scan only) timeout on the reception of replies from the servos (in s)
- **servos**: object which keys are the name of the joints and which values contain:
  - **id** (required): actuator's ID to its name (the one used in the controller list and in URDF)
  - **offset**: correction to be applied to the angle of the joint (in rad)
  - **command_interface**: the command mode (velocity or position)
  - **max_speed**: maximal allowed velocity (rad/s), **for now, works only for joints in position control**
- **default_command_interface**: if no **command_interface** is defined for a joint, this value is used instead

### Testing the hardware interface
If you want to use the sample launch files or to use one of the default controllers, please install with apt-get:

- `ros-YourDistro-ros-controllers` and
- `ros-YourDistro-joint-state-publisher`.

Have a look at the `launch/sample.launch` file. It will by default launch two feed-forward only controllers (one position and one velocity) and a virtual controller that publishes the states of the two actuators.

*Before starting* it, check `config/sample.yaml` for the `id` parameters, the `serial_interface` and `baudrate` settings. Once you are sure that it's correct, you can `roslaunch dynamixel_control_hw sample.launch`. By looking at the available topics, you should find two, for the commands the joints, and one for the joint state.

## Alternative software
If you know of an other software offering similar functionalities to this one, feel free to open an issue so that we can add it here.

## Support and contact
This software is developed as part of the ResiBots project. We do our best to keep it free of bug and to implement relevant features. Should you face an issue or have a suggestion, please open an issue.

## Projects using this software
- [ResiBots][]

## License
Unless stated otherwise, the license for this repository is CeCILL-C (see LICENSE and LICENSE.fr).

[ResiBots]: http://www.resibots.eu
[libdynamixel]: http://github.com/resibots/libdynamixel
[ROS]: http://www.ros.org/
[ROS control]: http://wiki.ros.org/ros_control
