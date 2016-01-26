#ifndef DYNAMIXEL_HARDWARE_INTERFACE
#define DYNAMIXEL_HARDWARE_INTERFACE

// ROS
#include <ros/ros.h>

// ROS control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// Library for access to the dynamixels
#include "dynamixel/dynamixel.hpp"

namespace dynamixel
{
    /** Hardware interface for a set of dynamixel actuators.

        This class fits in the ros_control framework for robot control.

        Warning, FIXME: due to the low-level nature of the dynamixel library in use, the
            position and velocities are returned as integers, as explained in the
            Dynamixel's documentation.
            same goes for the position commands are.
        Warning: this code is currently limited to joint-mode dynamixel actuators
    **/
    class DynamixelHardwareInterface : public hardware_interface::RobotHW
    {
    public:
        DynamixelHardwareInterface(const std::string& usb_serial_interface, int baudrate, std::map<byte_t, std::string> dynamixel_map);
        ~DynamixelHardwareInterface();

        /// Find all connected devices and register those refered in dynamixel_map in the
        /// hardware interface.
        void init();

        void read_joints();
        void write_joints();
    private:
        // not implemented
        DynamixelHardwareInterface(DynamixelHardwareInterface const&);

        // not implemented
        DynamixelHardwareInterface& operator=(DynamixelHardwareInterface const&);

        // ROS's hardware interface instances
        hardware_interface::JointStateInterface _jnt_state_interface;
        hardware_interface::PositionJointInterface _jnt_pos_interface;

        // Memory space shared with the controller
        // It reads here the latest robot's state and put here the next desired values
        std::vector<std::string> _joint_names;
        std::vector<double> _joint_commands; // target joint angle
        std::vector<double> _joint_angles; // actual joint angle
        std::vector<double> _joint_velocities; // actual joint velocity
        std::vector<double> _joint_efforts; // compulsory but not used

        // Dynamixel's low level controller
        Usb2Dynamixel _dynamixel_controller;

        // List of actuator's IDs (as we receive it from them)
        std::vector<byte_t> _dynamixel_ids;
        // Map from dynamixel ID to actuator's name
        std::map<byte_t, std::string> _dynamixel_map;

        // FIXME: make _read_duration parameterizable
        static const float _read_duration=0.02;
    };
}

# endif