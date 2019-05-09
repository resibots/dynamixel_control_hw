Testing the hardware interface
==================

This tutorial assumes that you have already been able to compile and install the package. If not, please refer to the related instructions in the :ref:`building/installation instructions <download_and_compilation>`.

For this example, we are using 2 dynamixel actuators : `MX28 <http://emanual.robotis.com/docs/en/dxl/mx/mx-28/>`__ .
It will by default **launch two feed-forward only controllers (one position and one velocity) and a virtual controller that publishes the states of the two actuators**.

1. Connect motor
    with your `USB2DYNAMIXEL <http://emanual.robotis.com/docs/en/parts/interface/usb2dynamixel/>`__ device or your favorite dynamixel interface.

2. Supply power
    USB2Dynamixel does not supply power to Dynamixel. Therefore, the power must be supplied separately to operate Dynamixel.
    We are recommanded to use : one laboratory power supply or `SMPS2Dynamixel <https://www.generationrobots.com/en/400867-smps2dynamixel-robotis.html>`__ .

Go inside your `libdynamixel <https://github.com/resibots/libdynamixel>`__ directory

3. Check serial interface
    open a terminal and simply type : ``ls /dev``.

    check your serial interface's name, in our case ``ttyUSB0``.

4. Check device's ID
    open a terminal and simply go into your libdynamixel directory : ``cd /home/USER/Resibots/libdynamixel``.
    Now, you can check with our tools interface if your motors and the libdynamixel work correctly :

``./build/src/tools/dynamixel -p /dev/ttyUSB0 -b 3000000 list``
        * ``-p`` device interface
        * ``-b`` dynamixel's baudrate (known)
        * ``list`` command to list all connected dynamixel

Go inside your  `dynamixel_control_hw <https://github.com/resibots/dynamixel_control_hw>`__ directory

5. Configuration file
    check the example configuration file: ``config/sample.yaml``
      * ``id`` parameters
      * ``serial_interface``
      * ``baudrate``

6. Launch file
    Check the ``launch/sample.launch`` file.

    It will by default launch two feed-forward only controllers (one position and one velocity) and a virtual controller that publishes the states of the two actuators.

    Once you are sure that it's correct, you can ``roslaunch dynamixel_control_hw sample.launch``.

7. Check topics and params
    rostopic list
    rosparam list
