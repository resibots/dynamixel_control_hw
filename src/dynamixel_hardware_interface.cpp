#include <dynamixel_hardware_interface/dynamixel_hardware_interface.hpp>

#include <stdexcept>

#include <math.h>

namespace dynamixel
{
    DynamixelHardwareInterface::DynamixelHardwareInterface(const std::string& usb_serial_interface, int baudrate, std::map<byte_t, std::string> dynamixel_map)
    try: _dynamixel_controller(usb_serial_interface, baudrate), _dynamixel_map(dynamixel_map)
    { // to catch eventual exception from initialisation list
    }
    catch (Error& e)
    {
        ROS_FATAL_STREAM("Caught an exception while trying to initialise the Dynamixels:\n"
            << e.msg());
        throw e;
    }

    DynamixelHardwareInterface::~DynamixelHardwareInterface()
    {
        // stop all actuators -> power them off ?
        // close dynamixel connexion
    }

    void DynamixelHardwareInterface::init()
    {
        // get the list of available actuators
        _dynamixel_controller.scan_ax12s();
        _dynamixel_ids = _dynamixel_controller.ax12_ids();
        ROS_DEBUG_STREAM(_dynamixel_ids.size() << "Dynamixel actuators are connected");

        _joint_commands.resize(_dynamixel_ids.size(), 0.0);
        _joint_angles.resize(_dynamixel_ids.size(), 0.0);
        _joint_velocities.resize(_dynamixel_ids.size(), 0.0);
        _joint_efforts.resize(_dynamixel_ids.size(), 0.0);

        try
        {
            for (unsigned i=0; i<_dynamixel_ids.size(); i++)
            {
                std::map<byte_t, std::string>::iterator dynamixel_iterator = _dynamixel_map.find(_dynamixel_ids[i]);
                if(dynamixel_iterator != _dynamixel_map.end()) // check that the actuator's name is in the map
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
                    dynamixel::Status status;
                    _dynamixel_controller.send(dynamixel::ax12::TorqueEnable(_dynamixel_ids[i],true));
                    _dynamixel_controller.recv(_read_duration, status);
                }
            }

            // register the hardware interfaces
            registerInterface(&_jnt_state_interface);
            registerInterface(&_jnt_pos_interface);
        }
        catch(const ros::Exception& e)
        {
            ROS_ERROR_STREAM("Could not initialize hardware interface:\n\tTrace: " << e.what());
            throw e;
        }
    }

    /** Copy joint's information to memory

        firslty queries the information from the dynamixels, then put it in private
        attributes, for use by a controller.

        Warning: do not get any information on torque
    **/
    void DynamixelHardwareInterface::read_joints()
    {
        try
        {
            for (unsigned i=0; i<_dynamixel_ids.size(); i++)
            {
                dynamixel::Status status;
                // current position
                _dynamixel_controller.send(dynamixel::ax12::GetPosition(_dynamixel_ids[i]));
                _dynamixel_controller.recv(_read_duration, status);
                _joint_angles[i] = status.decode16(); // TODO: convert value to SI units

                // current speed
                _dynamixel_controller.send(dynamixel::ax12::GetSpeed(_dynamixel_ids[i]));
                _dynamixel_controller.recv(_read_duration, status);
                _joint_velocities[i] = status.decode16(); // TODO: convert value to SI units
            }
        }
        catch (Error& e)
        {
            ROS_ERROR_STREAM("Caught a Dynamixel exception while getting the actuator's state:\n"
                << e.msg());
        }
    }

    /** Send new joint's target position to dynamixels

        takes the target position from memory (given by a controller) and sends
        them to the dynamixels.
    **/
    void DynamixelHardwareInterface::write_joints()
    {
        std::vector<int> command_int;
        command_int.resize(_dynamixel_ids.size());

        for(unsigned int i=0; i<_dynamixel_ids.size(); i++)
        {
            // TODO: enforce joint limits (from URDF ?)
            // TODO: actually convert from angle to tics
            command_int[i] = (int)(_joint_commands[i]*M_PI/2048.0);
        }

        try
        {
            dynamixel::Status status;
            _dynamixel_controller.send(dynamixel::ax12::SetPositions(_dynamixel_ids, command_int));
            _dynamixel_controller.recv(_read_duration, status);
        }
        catch (Error& e)
        {
            ROS_ERROR_STREAM("Caught a Dynamixel exception while sending new commands:\n"
                << e.msg());
        }
    }
}