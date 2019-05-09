Quick Start
===========

Dependencies
------------

Dependencies :

- `ROS kinetic <http://wiki.ros.org/kinetic>`__
- python2.x
- C++
- `libdynamixel <https://github.com/resibots/libdynamixel>`__
- `ROS control <https://github.com/ros-controls/ros_control>`__

.. note:: You need to store the installation path of `libdynamixel <https://github.com/resibots/libdynamixel>`__ in your environment variable inside your  ``~/.bashrc`` file  ``export LIBDYNAMIXEL=/home/USER/Resibots`` .
          Install the ros package ros_control. In apt-get, it is called ros-YourDistro-ros-control (where YourDistro must be changed by the ROS distribution's name, like indigo or jade).
          Clone the dynamixel_control_hw repository in your catkin workspace's source directory and run a catkin_make


ROS Dependencies :

- catkin
- rospy
- roscpp
- urdf
- std_msgs
- message_generation
- hardware_interface
- combined_robot_hw
- controller_manager
- ros-controllers
- joint-state-publisher

Building
--------

.. highlight:: shell

The build system for this package is **catkin** . Don't run away yet. Here is how we compile and install it :

.. note:: don't forget to install `ROS kinetic <http://wiki.ros.org/kinetic>`__ before


1. Clone
    simply clone the package `dynamixel_control_hw <https://github.com/resibots/dynamixel_control_hw>`__ into your catkin workspace source directory. ::

      cd $HOME/catkin_ws/src
      git clone https://github.com/resibots/dynamixel_control_hw.git

2. Compilation
    go into our workspace directory, then compile. ::

      cd ..
      catkin_make

3. Have fun
    Now, `dynamixel_control_hw <https://github.com/resibots/dynamixel_control_hw>`__ package should be correctly installed.

for more options and troubleshooting, see the :ref:`Compilation tutorial <download_and_compilation>`
