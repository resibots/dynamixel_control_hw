Architecture
============

In this document we will describe how the package is built.

Control loop
------------

The ``control_loop.hpp`` initializes controller manager (load, unload controller dynamically) then it loads rosparams (``loop_frequency`` and ``cycle_time_error_threshold``).

``update`` function reads data from Hardware interface, let the controller computes the new command (via the controller manager), then it send new command to hardware.


Hardware interface
------------------

``hardware_interface.hpp`` class implement an Hardware Interface for a set of dynamixel actuators.
This class fits in the `ros_control <http://wiki.ros.org/ros_control>`__ framework for robot control.
Different method are implemented :
  * init():
      Initialise the whole hardware interface. Set the essential parameters for communication with the hardware and find all connected devices and register those referred in dynamixel_map in the hardware interface.
  * read():
      Copy joint's information to memory, firstly queries the information from the dynamixels, then put it in private attributes, for use by a controller.
  * write():
      Send new joint's target position to dynamixels takes the target position from memory (given by a controller) and sends them to the dynamixels.

Methods used for initialisation :
  * ``_get_ros_parameters``
  * ``_load_urdf``
  * ``_find_servos``
  * ``_enable_and_configure_servo``
  * ``_register_joint_limits``


Dynamixel control_loop
----------------------

Run the hardware interface node, we run the ROS loop in a separate threads as external calls, such as service callbacks loading controllers, can't block the control loop.
It creates the hardware interface specific to your robot :
  1. retrieve configuration from rosparam
  2. initialize the hardware and interface it with ros_control
  3. Start the control loop
  4. wait until shutdown signal received
