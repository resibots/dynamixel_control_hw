#ifndef dynamixel_hardware_interface
#define dynamixel_hardware_interface

// ROS
#include <ros/ros.h>

// ROS control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// Library for access to the dynamixels
#include <dynamixel/dynamixel.hpp>

namespace dynamixel {
    /** Hardware interface for a set of dynamixel actuators.

        This class fits in the ros_control framework for robot control.

        Warning, FIXME: due to the low-level nature of the dynamixel library in use,
            the position and velocities are returned as integers, as explained in
            the Dynamixel's documentation.
            same goes for the position commands are.
        Warning: this code is currently limited to joint-mode dynamixel actuators
    **/
    class DynamixelHardwareInterface : public hardware_interface::RobotHW {
    public:
        /** Set the essential parameters for communication with the hardware.

            @param usb_serial_interface: name of the USB to serial interface;
                for example "/dev/ttyUSB0"
            @param baudrate: baud-rate for the serial communication with actuators (in bauds)
            @param dynamixel_timeout: timeout in seconds to wait for a message
                to arrive on seria bus
            @param dynamixel_map: map actuator's ID to its name (the one used
                in the controller list and in URDF)
            @param dynamixel_max_speed map from actuator IDs to maximal allowed
                velocity (the value is specific to the actuator's type)
            @param dynamixel_corrections map from actuator IDs to the correction
                to be applied to the angle
        **/
        DynamixelHardwareInterface(const std::string& usb_serial_interface, const int& baudrate,
            const float& read_timeout, std::map<long long int, std::string> dynamixel_map,
            std::map<long long int, long long int> dynamixel_max_speed, std::map<long long int, double> dynamixel_corrections);
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
        std::vector<double> _prev_commands;
        std::vector<double> _joint_commands; // target joint angle
        std::vector<double> _joint_angles; // actual joint angle
        std::vector<double> _joint_velocities; // actual joint velocity
        std::vector<double> _joint_efforts; // compulsory but not used

        // USB to serial connexion settings
        const std::string& _usb_serial_interface;
        const int _baudrate;
        const float _read_timeout; // in seconds
        // Dynamixel's low level controller
        dynamixel::controllers::Usb2Dynamixel _dynamixel_controller;

        // List of actuators (collected at init. from the actuators)
        using dynamixel_servo = std::shared_ptr<dynamixel::servos::BaseServo<dynamixel::protocols::Protocol1>>;
        std::vector<dynamixel_servo> _dynamixel_servos;
        // Map from dynamixel ID to actuator's name
        std::map<long long int, std::string> _dynamixel_map;
        // Map for max speed
        std::map<long long int, long long int> _dynamixel_max_speed;
        // Map for hardware corrections
        std::map<long long int, double> _dynamixel_corrections;
    };
}

#endif
