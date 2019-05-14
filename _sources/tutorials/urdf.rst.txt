.. _urdf:

URDF file
=========

the Unified Robot Description Format (URDF) is an XML format for representing a robot model.
You need to generate one specific to your robot.

You need to be care about ``joint_name``, they should have exactly the same name inside the :ref:`configuration file  <configuration>` .

Then, check ``joint_limits`` (hard and soft), changes it if needed.

Below an URDF example  of a 3 DoF robot arm ::

  <robot name="dummy_robot">
    <link name="base_link">
      <visual>
        <origin xyz="0 0 0.03" rpy="0 0 0"/>
        <geometry>
          <cylinder length="0.06" radius="0.04"/>
        </geometry>
      </visual>
    </link>
    <link name="middle_link">
      <visual>
        <origin xyz="0 0 0.13" rpy="0 0 0 "/>
        <geometry>
          <cylinder length="0.26" radius="0.04"/>
        </geometry>
      </visual>
    </link>
    <link name="second_middle_link">
      <visual>
        <origin xyz="0 0 0.13" rpy="0 0 0 "/>
        <geometry>
          <cylinder length="0.26" radius="0.04"/>
        </geometry>
      </visual>
    </link>
    <link name="end_link">
      <visual>
        <origin xyz="0 0 0.075" rpy="0 0 0 "/>
        <geometry>
          <cylinder length="0.15" radius="0.04"/>
        </geometry>
      </visual>
    </link>

    <joint name="arm_joint_1" type="revolute">
      <parent link="base_link"/>
      <child link="middle_link"/>
      <origin xyz="0 0 0.06" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>

      <!-- Joint limits -->
      <limit lower="-10.1415" upper="10.1415" velocity="10" effort="100"/>
      <!-- Soft limits -->
      <safety_controller k_position="3"
                      k_velocity="1000"
                      soft_lower_limit="-10.1415"
                      soft_upper_limit="10.1415" />
    </joint>

    <joint name="arm_joint_2" type="revolute">
      <parent link="middle_link"/>
      <child link="second_middle_link"/>
      <origin xyz="0 0 0.26" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>

      <!-- Joint limits -->
      <limit lower="-3.1415" upper="3.1415" velocity="10" effort="100"/>
      <!-- Soft limits -->
      <safety_controller k_position="3"
                      k_velocity="1000"
                      soft_lower_limit="-3.1415"
                      soft_upper_limit="3.1415" />
    </joint>

    <joint name="arm_joint_3" type="revolute">
      <parent link="second_middle_link"/>
      <child link="end_link"/>
      <origin xyz="0 0 0.26" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>

      <!-- Joint limits -->
      <limit lower="-3.1415" upper="3.1415" velocity="10" effort="100"/>
      <!-- Soft limits -->
      <safety_controller k_position="3"
                      k_velocity="1000"
                      soft_lower_limit="-3.1415"
                      soft_upper_limit="3.1415" />
    </joint>
  </robot>
