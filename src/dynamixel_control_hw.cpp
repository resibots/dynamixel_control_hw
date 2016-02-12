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
#include <sstream>

#include <dynamixel_control_hw/dynamixel_loop.hpp>
#include <dynamixel_control_hw/dynamixel_hardware_interface.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamixel_control_hw");
    ros::NodeHandle nh;

    // Get parameters for the hardware
    // -------------------------------

    ros::NodeHandle nhParams("~");
    std::string sub_namespace = nhParams.getNamespace();
    bool got_all_params = true;

    std::string usb_serial_interface;
    got_all_params &= nhParams.getParam("serial_interface", usb_serial_interface);
    int baudrate; // in bauds
    got_all_params &= nhParams.getParam("baudrate", baudrate);
    float dynamixel_timeout; // in seconds
    got_all_params &= nhParams.getParam("dynamixel_timeout", dynamixel_timeout);

    // Retrieve the map from joint name to hardware-related ID
    // It has to be inverted, putting the ID as a key, for later use
    std::map<dynamixel::byte_t, std::string> dynamixel_map;
    std::map<std::string, int> map_param; // temporary map, from parameter server
    got_all_params &= nhParams.getParam("hardware_mapping", map_param);
    std::map<std::string, int>::iterator map_param_i;
    for (map_param_i = map_param.begin(); map_param_i != map_param.end(); map_param_i++) {
        dynamixel_map[map_param_i->second] = map_param_i->first;
    }

    // Retrieve the map with angle corrections (ID: correction in ticks)
    std::map<dynamixel::byte_t, int> dynamixel_corrections;
    std::map<std::string, int> map_corrections; // temporary map, from parameter server
    nhParams.getParam("hardware_corrections", map_corrections);
    std::map<std::string, int>::iterator map_cor_i;
    for (map_cor_i = map_corrections.begin(); map_cor_i != map_corrections.end(); map_cor_i++) {
        int k;
        std::istringstream(map_cor_i->first) >> k;
        dynamixel_corrections[k] = map_cor_i->second;
    }

    if (!got_all_params) {
        std::string error_message = "One or more of the following parameters were not set:\n"
                                    "\t/" + sub_namespace + "/serial_interface /" + sub_namespace + "/baudrate"
                                                                                                    "/" + sub_namespace + "/dynamixel_timeout /" + sub_namespace + "/hardware_mapping";
        ROS_FATAL_STREAM(error_message);
        return 1;
    }

    // Run the hardware interface node
    // -------------------------------

    // We run the ROS loop in a separate thread as external calls, such
    // as service callbacks loading controllers, can block the (main) control loop
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Create the hardware interface specific to your robot
    boost::shared_ptr<dynamixel::DynamixelHardwareInterface>
        dynamixel_hw_interface = boost::make_shared<dynamixel::DynamixelHardwareInterface>(
            usb_serial_interface,
            baudrate,
            dynamixel_timeout,
            dynamixel_map,
            dynamixel_corrections);
    dynamixel_hw_interface->init();

    // Start the control loop
    dynamixel::DynamixelLoop control_loop(nh, dynamixel_hw_interface);

    // Wait until shutdown signal recieved
    ros::waitForShutdown();

    return 0;
}
