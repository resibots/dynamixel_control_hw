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
#include <unordered_map>

#include <dynamixel_control_hw/dynamixel_loop.hpp>
#include <dynamixel_control_hw/dynamixel_hardware_interface.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamixel_control_hw");
    ros::NodeHandle nh;

#ifdef PROTOCOL1
    ROS_INFO_STREAM("Dynamixel communication protocol : version 1");
    using Protocol = dynamixel::protocols::Protocol1;
#else
    ROS_INFO_STREAM("Dynamixel communication protocol : version 2");
    using Protocol = dynamixel::protocols::Protocol2;
#endif

    // Get parameters for the hardware
    // -------------------------------

    ros::NodeHandle nhParams("~");
    std::string sub_namespace = nhParams.getNamespace();
    bool got_all_params = true;

    std::string usb_serial_interface;
    got_all_params &= nhParams.getParam("serial_interface", usb_serial_interface);
    int baudrate; // in bauds
    got_all_params &= nhParams.getParam("baudrate", baudrate);
    double dynamixel_timeout; // in seconds
    got_all_params &= nhParams.getParam("dynamixel_timeout", dynamixel_timeout);
    double dynamixel_scanning; // in seconds
    bool dyn_scan = nhParams.getParam("dynamixel_scanning", dynamixel_scanning);
    if (!dyn_scan) {
        ROS_WARN_STREAM("Dynamixel scanning timeout parameter was not found. "
            << "Setting to default: 0.05s.");
        dynamixel_scanning = 0.05;
    }

    // Retrieve the map from joint name to hardware-related ID
    // It has to be inverted, putting the ID as a key, for later use
    std::unordered_map<Protocol::id_t, std::string> dynamixel_map;
    std::map<std::string, int> map_param; // temporary map, from parameter server
    got_all_params &= nhParams.getParam("hardware_mapping", map_param);
    std::map<std::string, int>::iterator map_param_i;
    for (map_param_i = map_param.begin(); map_param_i != map_param.end(); map_param_i++) {
        dynamixel_map[(Protocol::id_t)map_param_i->second] = map_param_i->first;
    }

    // Retrieve the map for max speed (ID: max speed)
    std::unordered_map<Protocol::id_t, double> dynamixel_max_speed_map;
    std::map<std::string, double> max_speed_param; // temporary map, from parameter server
    nhParams.getParam("max_speed", max_speed_param);
    std::map<std::string, double>::iterator max_speed_param_i;
    for (max_speed_param_i = max_speed_param.begin(); max_speed_param_i != max_speed_param.end(); max_speed_param_i++) {
        Protocol::id_t k;
        std::istringstream(max_speed_param_i->first) >> k;
        dynamixel_max_speed_map[k] = max_speed_param_i->second;
    }

    // Retrieve the map with angle corrections (ID: correction in radians)
    std::unordered_map<Protocol::id_t, double> dynamixel_corrections;
    std::map<std::string, double> map_corrections; // temporary map, from parameter server
    nhParams.getParam("hardware_corrections", map_corrections);
    std::map<std::string, double>::iterator map_cor_i;
    for (map_cor_i = map_corrections.begin(); map_cor_i != map_corrections.end(); map_cor_i++) {
        Protocol::id_t k;
        std::istringstream(map_cor_i->first) >> k;
        dynamixel_corrections[k] = map_cor_i->second;
    }

    if (!got_all_params) {
        std::string error_message = "One or more of the following parameters "
                                    "were not set:\n"
            + sub_namespace + "/serial_interface\n"
            + sub_namespace + "/baudrate\n"
            + sub_namespace + "/dynamixel_timeout\n"
            + sub_namespace + "/hardware_mapping\n"
            + sub_namespace + "/max_speed\n"
            + sub_namespace + "/hardware_corrections";
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
    std::shared_ptr<dynamixel::DynamixelHardwareInterface<Protocol>>
        dynamixel_hw_interface = std::make_shared<dynamixel::DynamixelHardwareInterface<Protocol>>(
            usb_serial_interface,
            baudrate,
            dynamixel_timeout,
            dynamixel_scanning,
            dynamixel_map,
            dynamixel_max_speed_map,
            dynamixel_corrections);
    dynamixel_hw_interface->init();

    // Start the control loop
    dynamixel::DynamixelLoop<Protocol> control_loop(nh, dynamixel_hw_interface);

    // Wait until shutdown signal recieved
    ros::waitForShutdown();

    return 0;
}
