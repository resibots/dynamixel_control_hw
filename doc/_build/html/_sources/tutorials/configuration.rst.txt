.. _configuration:

Configuration file
==================

The configuration file is the main user file to set different controllers and parameters.
The configuration file is a YML file located in ``config/``.


1. declare your different dynamixel controllers

  * name of your controller
  * type : joint state, position, velocity, trajectory
  * joint : set joint name (same as in urdf) ::

      dynamixel_controllers:

        joint_state_controller:
          type: joint_state_controller/JointStateController
          publish_rate: 10

        traj_controller:
          type: position_controllers/JointTrajectoryController
          joints:
            - JOINT_NAME1

        vel_controller:
          type: velocity_controllers/JointVelocityController
          joint: JOINT_NAME2


2. declare your dynamixel control parameters

  * ``serial interface``: path to the USB to serial interface for example ``/dev/ttyUSB0``
  * ``loop_frequency``: frequency at which the control loop will run (in Hz)
  * ``cycle_time_error_threshold``: how much delay is tolerated on the control loop (in s)
  * ``baudrate``: baud-rate for the serial communication with actuators (in bauds)
  * ``read_timeout``: timeout on the reception of replies from the servos (in s)
  * ``read_timeout``: (for the scan only) timeout on the reception of replies from the servos (in s)
  * ``servos``: object which keys are the name of the joints and which values contain:

    * ``id`` (required): actuator's ID to its name (the one used in the controller list and in URDF)
    * ``offset``: correction to be applied to the angle of the joint (in rad)
    * ``command_interface``: the command mode (velocity or position)
    * ``max_speed``: maximal allowed velocity (rad/s), for now, works only for joints in position control
    * ``reverse``: clockwise and anticlockwise
  * ``default_command_interface``: if no command_interface is defined for a joint, this value is used instead

Below one example of configuration file, see :ref:`sample <simple_example>` example ::

  dynamixel_controllers:

    joint_state_controller:
      type: joint_state_controller/JointStateController
      publish_rate: 10

    traj_controller:
      type: position_controllers/JointTrajectoryController
      joints:
        - arm_joint_2

    vel_controller:
      type: velocity_controllers/JointVelocityController
      joint: arm_joint_1

  dynamixel_control:
    serial_interface: /dev/ttyUSB0
    loop_frequency: 500
    cycle_time_error_threshold: 0.002
    baudrate: 3000000 # in bauds
    read_timeout: 0.005 # in seconds
    scan_timeout: 0.005 # in seconds
    # Cofniguration of the servos
    servos:
      arm_joint_1:
        # hardware ID of the actuator
        id: 2
        offset: 3.14159265359
        # offset to be added, in radians, to the position of an actuator
        #max-speed: 4.0 # in rad
        command_interface: velocity
      arm_joint_2:
        id: 1
        # max joint speed
        # it shoud NOT be used for actuators in velocity mode
        offset: 3.1 # in rad/s
        # this actuator is set in reverse mode for the speed and position
        #reverse: false
    # default mode
    default_command_interface: position
