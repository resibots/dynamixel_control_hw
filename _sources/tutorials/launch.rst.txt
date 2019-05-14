.. _launch:

Launch file
===========

If you want to use the sample launch files or to use one of the default controllers, please install with apt-get: ::

    apt-get install ros-kinetic-ros-controllers
    apt-get install ros-kinetic-joint-state-publisher
    apt-get install ros-kinetic-position-controllers
    apt-get install ros-kinetic-velocity-controllers
    apt-get install ros-kinetic-joint-trajectory-controller
.. note:: change ``kinetic`` by your ros distro name if needed

Have a look at the ``launch/sample.launch`` file. It will by default launch two feed-forward only controllers (one position and one velocity) and a virtual controller that publishes the states of the two actuators.
::
  <?xml version="1.0"?>

  <launch>
    <arg name="fake_robot" default="false"
           doc="When set to true, assume a fake robot for visualization in rviz,
                disabling hardware interface."/>

    <!-- Parameters for the hardware interface and controllers -->
    <rosparam file="$(find dynamixel_control_hw)/config/sample.yaml"/>

    <!-- URDF robot description -->
    <param name="robot_description"
           command="cat $(find dynamixel_control_hw)/urdf/sample.urdf" />
    <!-- Publish robot's state in the tf tree -->
    <node pkg="robot_state_publisher" type="robot_state_publisher"
          name="rob_state_pub" />

    <!-- To move the dummy robot with a gui, bypassing the controller manager -->
    <node if="$(arg fake_robot)" pkg="joint_state_publisher"
      type="joint_state_publisher" name="dummy_robot" />

    <!-- launch our hardware interface -->
    <group unless="$(arg fake_robot)">
      <arg name="protocol_version" default="p1"
           doc="Choses the dynamixel protocol variant to be used (p1 or p2)"/>
      <node pkg="dynamixel_control_hw"
            type="dynamixel_control_$(arg protocol_version)"
            name="dynamixel_control" output="screen" clear_params="true"
            required="true"/>

      <!-- Start a controller for our dummy robot -->
      <node name="controller" pkg="controller_manager" type="spawner"
            respawn="false" output="screen" clear_params="true"
            args="/dynamixel_controllers/traj_controller
                  /dynamixel_controllers/vel_controller
                  /dynamixel_controllers/joint_state_controller" />
    </group>
  </launch>
