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
#include <unordered_map>

#include <dynamixel_control_hw/dynamixel_hardware_interface.hpp>
#include <dynamixel_control_hw/dynamixel_loop.hpp>

// for dynamixel::OperatingMode
#include <dynamixel/servos/base_servo.hpp>

// to parse parameters
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

dynamixel::OperatingMode get_mode(const std::string& mode_string, std::string name);

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
    double read_timeout; // in seconds
    got_all_params &= nhParams.getParam("read_timeout", read_timeout);
    double scan_timeout; // in seconds
    bool dyn_scan = nhParams.getParam("scan_timeout", scan_timeout);
    if (!dyn_scan) {
        ROS_WARN_STREAM("Dynamixel scanning timeout parameter was not found. "
            << "Setting to default: 0.05s.");
        scan_timeout = 0.05;
    }
    std::string default_command_interface;
    bool has_default_command_interface = nhParams.getParam(
        "default_command_interface", default_command_interface);

    // Map from joint name to hardware-related ID
    std::unordered_map<Protocol::id_t, std::string> dynamixel_map;
    // Map for command interface (ID: velocity/position)
    std::unordered_map<Protocol::id_t, dynamixel::OperatingMode> dynamixel_c_mode_map;
    // Map with angle corrections (ID: correction in radians)
    std::unordered_map<Protocol::id_t, double> dynamixel_corrections;
    // Map for max speed (ID: max speed)
    std::unordered_map<Protocol::id_t, double> dynamixel_max_speed;

    // Retrieve the srevo parameter and fill maps above with its content.
    std::unordered_map<Protocol::id_t, std::string> servos_map;
    XmlRpc::XmlRpcValue servos_param; // temporary map, from parameter server
    got_all_params &= nhParams.getParam("servos", servos_param);
    if (got_all_params &= nhParams.getParam("servos", servos_param)) {
        ROS_ASSERT(servos_param.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        try {
            for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = servos_param.begin(); it != servos_param.end(); ++it) {
                ROS_DEBUG_STREAM("servo: " << (std::string)(it->first));

                Protocol::id_t id;
                if (it->second.hasMember("id")) {
                    id = static_cast<int>(servos_param[it->first]["id"]);
                    ROS_DEBUG_STREAM("\tid: " << servos_param[it->first]["id"]);
                    dynamixel_map[id] = it->first;
                }
                else {
                    ROS_ERROR_STREAM("Actuator " << it->first
                                                 << " has no associated servo ID.");
                }

                if (it->second.hasMember("offset")) {
                    ROS_DEBUG_STREAM("\toffset: "
                        << servos_param[it->first]["offset"]);
                    dynamixel_corrections[id]
                        = static_cast<double>(servos_param[it->first]["offset"]);
                }
                if (it->second.hasMember("max_speed")) {
                    ROS_DEBUG_STREAM("\tmax_speed: "
                        << servos_param[it->first]["max_speed"]);
                    dynamixel_max_speed[id]
                        = static_cast<double>(servos_param[it->first]["max_speed"]);
                }

                if (it->second.hasMember("command_interface")) {
                    std::string mode_string
                        = static_cast<std::string>(servos_param[it->first]["command_interface"]);
                    ROS_DEBUG_STREAM("\tcommand_interface: " << mode_string);

                    dynamixel_c_mode_map[id] = get_mode(mode_string, it->first);
                }
                else if (has_default_command_interface) {
                    ROS_DEBUG_STREAM("\tcommand_interface: "
                        << default_command_interface << " (default)");
                    dynamixel_c_mode_map[id]
                        = get_mode(default_command_interface, it->first);
                }
                else {
                    ROS_ERROR_STREAM("A command interface (speed or position) "
                        << "should be declared for the actuator " << it->first
                        << " or a default one should be defined with the parameter "
                        << "'default_command_interface'.");
                }
            }
        }
        catch (XmlRpc::XmlRpcException& e) {
            ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
                << "configuration: " << e.getMessage() << ".\n"
                << "Please check the configuration, particularly parameter types.");
            return 1;
        }
    }

    if (!got_all_params) {
        std::string error_message = "One or more of the following parameters "
                                    "were not set:\n"
            + sub_namespace + "/serial_interface\n"
            + sub_namespace + "/baudrate\n"
            + sub_namespace + "/read_timeout\n"
            + sub_namespace + "/servos";
        ROS_FATAL_STREAM(error_message);
        return 1;
    }

    // Run the hardware interface node
    // -------------------------------

    // We run the ROS loop in a separate thread as external calls, such
    // as service callbacks loading controllers, can block the (main) control loop

    ros::AsyncSpinner spinner(2);
    spinner.start();

    try {
        // Create the hardware interface specific to your robot
        std::shared_ptr<dynamixel::DynamixelHardwareInterface<Protocol>>
            dynamixel_hw_interface = std::make_shared<dynamixel::DynamixelHardwareInterface<Protocol>>(
                usb_serial_interface,
                baudrate,
                read_timeout,
                scan_timeout,
                dynamixel_map,
                dynamixel_c_mode_map,
                dynamixel_max_speed,
                dynamixel_corrections);
        dynamixel_hw_interface->init();

        // Start the control loop
        dynamixel::DynamixelLoop<Protocol> control_loop(nh, dynamixel_hw_interface);

        // Wait until shutdown signal recieved
        ros::waitForShutdown();
    }
    catch (const ros::Exception& e) {
        ROS_FATAL_STREAM("Error in the hardware interface:\n"
            << "\tTrace: " << e.what());
        return 1;
    }
    catch (const dynamixel::errors::Error& e) {
        ROS_FATAL_STREAM("Error in the hardware interface:\n"
            << "\tTrace: " << e.msg());
        return 1;
    }

    return 0;
}

/** Convert a string to an operating mode for a Dynamixel servo

    @param mode_string name of the operating mode (either velocity or position)
    @param nam name of the joint
    @return dynamixel::OperatingMode::unknown if mode_string is not recognized
**/
dynamixel::OperatingMode get_mode(const std::string& mode_string, std::string name)
{
    dynamixel::OperatingMode mode;
    if ("velocity" == mode_string)
        mode = dynamixel::OperatingMode::wheel;
    else if ("position" == mode_string)
        mode = dynamixel::OperatingMode::joint;
    else {
        mode = dynamixel::OperatingMode::unknown;
        ROS_ERROR_STREAM("The command mode " << mode_string
                                             << " is not available (actuator "
                                             << name << ")");
    }

    return mode;
}