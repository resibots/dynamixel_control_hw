/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, University of Colorado, Boulder
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Univ of CO, Boulder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Original Author: Dave Coleman (https://github.com/davetcoleman/ros_control_boilerplate) */


#include <dynamixel_hardware_interface/dynamixel_loop.hpp>
#include <dynamixel_hardware_interface/dynamixel_hardware_interface.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamixel_control_renewed");
    ros::NodeHandle nh;

    std::string usb_serial_interface = "/dev/ttyUSB0";
    int baudrate = 0;
    std::map<dynamixel::byte_t, std::string> dynamixel_map;
    dynamixel_map[1] = "first";

    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control loop
    ros::MultiThreadedSpinner spinner(2);
    // TODO: use the following instead, as soon as we use ROS Jade
    // ros::AsyncSpinner spinner(2);
    // spinner.start();

    // Create the hardware interface specific to your robot
    boost::shared_ptr<dynamixel::DynamixelHardwareInterface>
        dynamixel_hw_interface (new dynamixel::DynamixelHardwareInterface(
            usb_serial_interface,
            baudrate,
            dynamixel_map));
    dynamixel_hw_interface->init();
    // Start the control loop

    dynamixel::DynamixelLoop control_loop(nh, dynamixel_hw_interface);

    // Wait until shutdown signal recieved
    spinner.spin();
    // TODO: use the following instead, as soon as we use ROS Jade
    // ros::waitForShutdown();

    return 0;
}