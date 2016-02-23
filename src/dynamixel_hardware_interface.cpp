#include <dynamixel_control_hw/dynamixel_hardware_interface.hpp>

#include <stdexcept>
#include <limits>

#include <math.h>

namespace dynamixel {
    DynamixelHardwareInterface::DynamixelHardwareInterface(const std::string& usb_serial_interface,
        const int& baudrate, const float& dynamixel_timeout, std::map<long long int, std::string> dynamixel_map, std::map<long long int, long long int> dynamixel_max_speed,
        std::map<long long int, double> dynamixel_corrections)
        : _dynamixel_map(dynamixel_map), _dynamixel_max_speed(dynamixel_max_speed), _dynamixel_corrections(dynamixel_corrections), _usb_serial_interface(usb_serial_interface), _baudrate(get_baudrate(baudrate)), _read_timeout(dynamixel_timeout)
    {
    }

    DynamixelHardwareInterface::~DynamixelHardwareInterface()
    {
        // stop all actuators
        try {
            for (auto dynamixel_servo : _dynamixel_servos) {
                dynamixel::StatusPacket<dynamixel::protocols::Protocol1> status;
                _dynamixel_controller.send(dynamixel_servo->set_torque_enable(0));
                _dynamixel_controller.recv(status);
            }
        }
        catch (dynamixel::errors::Error& e) {
            ROS_FATAL_STREAM("Caught a Dynamixel exception while trying to power them off:\n"
                << e.msg());
            throw e;
        }
    }

    void DynamixelHardwareInterface::init()
    {
        // get the list of available actuators
        try {
            _dynamixel_controller.open_serial(_usb_serial_interface, _baudrate, _read_timeout);
            _dynamixel_servos = dynamixel::auto_detect<dynamixel::protocols::Protocol1>(_dynamixel_controller);
        }
        catch (dynamixel::errors::Error& e) {
            ROS_FATAL_STREAM("Caught a Dynamixel exception while trying to initialise them:\n"
                << e.msg());
            throw e;
        }

        _prev_commands.resize(_dynamixel_servos.size(), 0.0);
        _joint_commands.resize(_dynamixel_servos.size(), 0.0);
        _joint_angles.resize(_dynamixel_servos.size(), 0.0);
        _joint_velocities.resize(_dynamixel_servos.size(), 0.0);
        _joint_efforts.resize(_dynamixel_servos.size(), 0.0);

        // declare all available actuators to the control manager, provided a
        // name has been given for them
        // also enable the torque output on the actuators (sort of power up)
        try {
            for (unsigned i = 0; i < _dynamixel_servos.size(); i++) {
                std::map<long long int, std::string>::iterator dynamixel_iterator = _dynamixel_map.find(_dynamixel_servos[i]->id());
                if (dynamixel_iterator != _dynamixel_map.end()) // check that the actuator's name is in the map
                {
                    // give the memory address for the information on joint angle,
                    // velocity and effort to ros_control
                    hardware_interface::JointStateHandle state_handle(
                        dynamixel_iterator->second,
                        &_joint_angles[i],
                        &_joint_velocities[i],
                        &_joint_efforts[i]);
                    _jnt_state_interface.registerHandle(state_handle);

                    // tell ros_control the in-memory address to change to set new
                    // position goal for the actuator
                    hardware_interface::JointHandle pos_handle(
                        _jnt_state_interface.getHandle(dynamixel_iterator->second),
                        &_joint_commands[i]);
                    _jnt_pos_interface.registerHandle(pos_handle);

                    // enable the actuator
                    dynamixel::StatusPacket<dynamixel::protocols::Protocol1> status;
                    _dynamixel_controller.send(_dynamixel_servos[i]->set_torque_enable(1));
                    _dynamixel_controller.recv(status);

                    if (_dynamixel_max_speed.size() != 0) {
                        dynamixel::StatusPacket<dynamixel::protocols::Protocol1> status;
                        _dynamixel_controller.send(_dynamixel_servos[i]->set_moving_speed(_dynamixel_max_speed[_dynamixel_servos[i]->id()]));
                        _dynamixel_controller.recv(status);
                    }
                }
            }

            // register the hardware interfaces
            registerInterface(&_jnt_state_interface);
            registerInterface(&_jnt_pos_interface);
        }
        catch (const ros::Exception& e) {
            ROS_ERROR_STREAM("Could not initialize hardware interface:\n\tTrace: " << e.what());
            throw e;
        }
        catch (dynamixel::errors::Error& e) {
            ROS_FATAL_STREAM("Caught a Dynamixel exception while trying to enable them:\n"
                << e.msg());
            throw e;
        }

        // At startup robot should keep the pose it has
        read_joints();

        for (unsigned i = 0; i < _dynamixel_servos.size(); i++) {
            _joint_commands[i] = _joint_angles[i];
        }
    }

    /** Copy joint's information to memory

        firslty queries the information from the dynamixels, then put it in private
        attributes, for use by a controller.

        Warning: do not get any information on torque
    **/
    void DynamixelHardwareInterface::read_joints()
    {
        for (unsigned i = 0; i < _dynamixel_servos.size(); i++) {
            std::map<long long int, std::string>::iterator dynamixel_iterator = _dynamixel_map.find(_dynamixel_servos[i]->id());
            if (dynamixel_iterator == _dynamixel_map.end())
                continue;
            dynamixel::StatusPacket<dynamixel::protocols::Protocol1> status;
            try {
                // current position
                _dynamixel_controller.send(_dynamixel_servos[i]->get_present_position_angle());
                _dynamixel_controller.recv(status);
            }
            catch (dynamixel::errors::Error& e) {
                ROS_ERROR_STREAM("Caught a Dynamixel exception while getting  " << _dynamixel_map[_dynamixel_servos[i]->id()] << "'s position\n" << e.msg());
            }
            if (status.valid()) {
                _joint_angles[i] = _dynamixel_servos[i]->parse_present_position_angle(status);
                if (_dynamixel_corrections.size() != 0) {
                    _joint_angles[i] -= _dynamixel_corrections[_dynamixel_servos[i]->id()];
                }
            }
            else {
                ROS_WARN_STREAM("Did not receive any data when reading " << _dynamixel_map[_dynamixel_servos[i]->id()] << "'s position");
            }

            // TO-DO: Make it work in V2 libdynamixel
            // dynamixel::StatusPacket<dynamixel::protocols::Protocol1> status_speed;
            // // current speed
            // _dynamixel_controller.send(_dynamixel_servos[i]->get_present_speed());
            // _dynamixel_controller.recv(status_speed);
            // if (status_speed.valid()) {
            //     _joint_velocities[i] = _dynamixel_servos[i]->parse_present_speed(status_speed);
            //     // <1024 ccw, >=1024 cw, 0.11rpm is the unit
            //     // we convert it to rad/s
            //     if (_joint_velocities[i] < 1024) {
            //         _joint_velocities[i] *= 0.01151917305;
            //     }
            //     else {
            //         _joint_velocities[i] = (_joint_velocities[i] - 1024) * -0.01151917305;
            //     }
            // }
            // else {
            //     ROS_WARN_STREAM("Did not receive any data when reading " << _dynamixel_map[_dynamixel_servos[i]->id()] << "'s velocity");
            // }
        }
    }

    /** Send new joint's target position to dynamixels

        takes the target position from memory (given by a controller) and sends
        them to the dynamixels.
    **/
    void DynamixelHardwareInterface::write_joints()
    {

        for (unsigned int i = 0; i < _dynamixel_servos.size(); i++) {
            std::map<long long int, std::string>::iterator dynamixel_iterator = _dynamixel_map.find(_dynamixel_servos[i]->id());
            if (dynamixel_iterator == _dynamixel_map.end())
                continue;
            // Sending commands only when needed
            if (std::abs(_joint_commands[i] - _prev_commands[i]) < std::numeric_limits<double>::epsilon())
                continue;
            _prev_commands[i] = _joint_commands[i];
            try {
                dynamixel::StatusPacket<dynamixel::protocols::Protocol1> status;
                _dynamixel_controller.send(_dynamixel_servos[i]->reg_goal_position_angle(_joint_commands[i] + ((_dynamixel_corrections.size() != 0) ? _dynamixel_corrections[_dynamixel_servos[i]->id()] : 0)));
                _dynamixel_controller.recv(status);
            }
            catch (dynamixel::errors::Error& e) {
                ROS_ERROR_STREAM("Caught a Dynamixel exception while sending new commands:\n"
                    << e.msg());
            }
        }

        try {
            _dynamixel_controller.send(dynamixel::instructions::Action<dynamixel::protocols::Protocol1>(dynamixel::protocols::Protocol1::broadcast_id));
        }
        catch (dynamixel::errors::Error& e) {
            ROS_ERROR_STREAM("Caught a Dynamixel exception while sending new commands:\n"
                << e.msg());
        }
    }
}
