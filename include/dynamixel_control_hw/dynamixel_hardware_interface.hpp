#ifndef dynamixel_hardware_interface
#define dynamixel_hardware_interface

#include <limits>
#include <math.h>
#include <stdexcept>
#include <unordered_map>

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

        Warning: this code is currently limited to joint-mode dynamixel actuators
    **/
    template <class Protocol>
    class DynamixelHardwareInterface : public hardware_interface::RobotHW {
    public:
        // Actuator's id type
        using id_t = typename Protocol::id_t;

        /** Set the essential parameters for communication with the hardware.

            @param usb_serial_interface: name of the USB to serial interface;
                for example "/dev/ttyUSB0"
            @param baudrate: baud-rate for the serial communication with actuators
                (in bauds)
            @param dynamixel_timeout: timeout in seconds to wait for a message
                to arrive on serial bus
            @param dynamixel_map: map actuator's ID to its name (the one used
                in the controller list and in URDF)
            @param dynamixel_c_mode_map: map actuator's ID to the command
                mode, as defined in libdynamixel dynamixel::OperatingMode
            @param dynamixel_max_speed map from actuator IDs to maximal allowed
                velocity (the value is specific to the actuator's type)
            @param dynamixel_corrections map from actuator IDs to the correction
                to be applied to the angle
        **/
        DynamixelHardwareInterface(const std::string& usb_serial_interface,
            const int& baudrate,
            const float& read_timeout,
            const float& scan_timeout,
            std::unordered_map<id_t, std::string> dynamixel_map,
            std::unordered_map<id_t, OperatingMode> dynamixel_c_mode_map,
            std::unordered_map<id_t, double> dynamixel_max_speed,
            std::unordered_map<id_t, double> dynamixel_corrections)
            : _usb_serial_interface(usb_serial_interface),
              _baudrate(get_baudrate(baudrate)),
              _read_timeout(read_timeout),
              _scan_timeout(scan_timeout),
              _dynamixel_map(dynamixel_map),
              _c_mode_map(dynamixel_c_mode_map),
              _dynamixel_max_speed(dynamixel_max_speed),
              _dynamixel_corrections(dynamixel_corrections)
        {
        }

        ~DynamixelHardwareInterface();

        /** Find all connected devices and register those referred in dynamixel_map
            in the hardware interface.
        **/
        void init();

        /** Copy joint's information to memory

            firstly queries the information from the dynamixels, then put it in
            private attributes, for use by a controller.

            Warning: do not get any information on torque
        **/
        void read_joints();

        /** Send new joint's target position to dynamixels

            takes the target position from memory (given by a controller) and sends
            them to the dynamixels.
        **/
        void write_joints();

    private:
        using dynamixel_servo = std::shared_ptr<dynamixel::servos::BaseServo<Protocol>>;

        DynamixelHardwareInterface(DynamixelHardwareInterface<Protocol> const&) = delete;
        DynamixelHardwareInterface& operator=(DynamixelHardwareInterface<Protocol> const&) = delete;

        void _find_servos();
        void _enable_and_configure_servo(dynamixel_servo servo, OperatingMode mode);

        // ROS's hardware interface instances
        hardware_interface::JointStateInterface _jnt_state_interface;
        hardware_interface::PositionJointInterface _jnt_pos_interface;
        hardware_interface::VelocityJointInterface _jnt_vel_interface;

        // Memory space shared with the controller
        // It reads here the latest robot's state and put here the next desired values
        std::vector<double> _prev_commands;
        std::vector<double> _joint_commands; // target joint angle or speed
        std::vector<double> _joint_angles; // actual joint angle
        std::vector<double> _joint_velocities; // actual joint velocity
        std::vector<double> _joint_efforts; // compulsory but not used

        // USB to serial connexion settings
        const std::string& _usb_serial_interface;
        const int _baudrate;
        const float _read_timeout, _scan_timeout; // in seconds
        // Dynamixel's low level controller
        dynamixel::controllers::Usb2Dynamixel _dynamixel_controller;

        // List of actuators (collected at init. from the actuators)
        std::vector<dynamixel_servo> _servos;
        // Map from dynamixel ID to actuator's name
        std::unordered_map<id_t, std::string> _dynamixel_map;
        // Map from dynamixel ID to actuator's command interface (velocity/position)
        std::unordered_map<id_t, OperatingMode> _c_mode_map;
        // Map for max speed
        std::unordered_map<id_t, double> _dynamixel_max_speed;
        // Map for hardware corrections
        std::unordered_map<id_t, double> _dynamixel_corrections;
    };

    template <class Protocol>
    DynamixelHardwareInterface<Protocol>::~DynamixelHardwareInterface()
    {
        // stop all actuators
        try {
            for (auto dynamixel_servo : _servos) {
                dynamixel::StatusPacket<Protocol> status;
                _dynamixel_controller.send(dynamixel_servo->set_torque_enable(0));
                _dynamixel_controller.recv(status);
            }
        }
        catch (dynamixel::errors::Error& e) {
            ROS_FATAL_STREAM("Caught a Dynamixel exception while trying to "
                << "power them off:\n"
                << e.msg());
            throw e;
        }
    }

    template <class Protocol>
    void DynamixelHardwareInterface<Protocol>::init()
    {
        _find_servos();

        _prev_commands.resize(_servos.size(), 0.0);
        _joint_commands.resize(_servos.size(), 0.0);
        _joint_angles.resize(_servos.size(), 0.0);
        _joint_velocities.resize(_servos.size(), 0.0);
        _joint_efforts.resize(_servos.size(), 0.0);

        // declare all available actuators to the control manager, provided a
        // name has been given for them
        // also enable the torque output on the actuators (sort of power up)
        try {
            for (unsigned i = 0; i < _servos.size(); i++) {
                id_t id = _servos[i]->id();

                // check that the actuator's name is in the map
                typename std::unordered_map<id_t, std::string>::iterator dynamixel_iterator
                    = _dynamixel_map.find(id);
                if (dynamixel_iterator != _dynamixel_map.end()) {
                    // tell ros_control the in-memory addresses where to read the
                    // information on joint angle, velocity and effort
                    hardware_interface::JointStateHandle state_handle(
                        dynamixel_iterator->second,
                        &_joint_angles[i],
                        &_joint_velocities[i],
                        &_joint_efforts[i]);
                    _jnt_state_interface.registerHandle(state_handle);

                    // check that the actuator control mode matches the declared
                    // command interface (position/velocity)
                    OperatingMode hardware_mode
                        = operating_mode<Protocol>(_dynamixel_controller, id);
                    typename std::unordered_map<id_t, OperatingMode>::iterator c_mode_map_i
                        = _c_mode_map.find(id);
                    if (c_mode_map_i != _c_mode_map.end()) {
                        if (c_mode_map_i->second != hardware_mode) {
                            ROS_ERROR_STREAM("The command interface declared "
                                << mode2str(c_mode_map_i->second)
                                << " for joint " << dynamixel_iterator->second
                                << " but is set to " << mode2str(hardware_mode)
                                << " in hardware. Disabling this joint.");
                            _c_mode_map[id] = OperatingMode::unknown;
                        }
                    }
                    else {
                        ROS_ERROR_STREAM("The command interface was not defined "
                            << "for joint " << dynamixel_iterator->second
                            << ". Disabling this joint.");
                        _c_mode_map[id] = OperatingMode::unknown;
                    }

                    // tell ros_control the in-memory address to change to set new
                    // position or velocity goal for the actuator (depending on
                    // hardware_mode)
                    hardware_interface::JointHandle cmd_handle(
                        _jnt_state_interface.getHandle(dynamixel_iterator->second),
                        &_joint_commands[i]);
                    if (OperatingMode::joint == hardware_mode) {
                        _jnt_pos_interface.registerHandle(cmd_handle);
                    }
                    else if (OperatingMode::wheel == hardware_mode) {
                        _jnt_vel_interface.registerHandle(cmd_handle);
                    }
                    else if (OperatingMode::unknown != hardware_mode)
                        ROS_ERROR_STREAM("Servo " << id << " was not initialised "
                                                  << "(operating mode "
                                                  << mode2str(hardware_mode)
                                                  << " is not supported)");

                    // enable torque output on the servo and set its configuration
                    // including max speed
                    _enable_and_configure_servo(_servos[i], hardware_mode);
                }
                else {
                    ROS_WARN_STREAM("Servo " << id << " was not initialised "
                                             << "(not found in the parameters)");
                }
            }

            // register the hardware interfaces
            registerInterface(&_jnt_state_interface);
            registerInterface(&_jnt_pos_interface);
            registerInterface(&_jnt_vel_interface);
        }
        catch (const ros::Exception& e) {
            ROS_ERROR_STREAM("Could not initialize hardware interface:\n"
                << "\tTrace: " << e.what());
            throw e;
        }

        // At startup robot should keep the pose it has
        read_joints();

        for (unsigned i = 0; i < _servos.size(); i++) {
            OperatingMode mode = _c_mode_map[_servos[i]->id()];
            if (OperatingMode::joint == mode)
                _joint_commands[i] = _joint_angles[i];
            else if (OperatingMode::wheel == mode)
                _joint_commands[i] = 0;
        }
    }

    template <class Protocol>
    void DynamixelHardwareInterface<Protocol>::read_joints()
    {
        for (unsigned i = 0; i < _servos.size(); i++) {
            dynamixel::StatusPacket<Protocol> status;
            try {
                // current position
                _dynamixel_controller.send(_servos[i]->get_present_position_angle());
                _dynamixel_controller.recv(status);
            }
            catch (dynamixel::errors::Error& e) {
                ROS_ERROR_STREAM("Caught a Dynamixel exception while getting  "
                    << _dynamixel_map[_servos[i]->id()] << "'s position\n"
                    << e.msg());
            }
            if (status.valid()) {
                try {
                    _joint_angles[i] = _servos[i]->parse_present_position_angle(status);
                }
                catch (dynamixel::errors::Error& e) {
                    ROS_ERROR_STREAM("Unpack exception while getting  "
                        << _dynamixel_map[_servos[i]->id()] << "'s position\n"
                        << e.msg());
                }

                // Apply angle correction to joint, if any
                typename std::unordered_map<id_t, double>::iterator dynamixel_corrections_iterator
                    = _dynamixel_corrections.find(_servos[i]->id());
                if (dynamixel_corrections_iterator != _dynamixel_corrections.end()) {
                    _joint_angles[i] -= dynamixel_corrections_iterator->second;
                }
            }
            else {
                ROS_WARN_STREAM("Did not receive any data when reading "
                    << _dynamixel_map[_servos[i]->id()] << "'s position");
            }

            dynamixel::StatusPacket<Protocol> status_speed;
            try {
                // current speed
                _dynamixel_controller.send(_servos[i]->get_present_speed());
                _dynamixel_controller.recv(status_speed);
            }
            catch (dynamixel::errors::Error& e) {
                ROS_ERROR_STREAM("Caught a Dynamixel exception while getting  "
                    << _dynamixel_map[_servos[i]->id()] << "'s velocity\n"
                    << e.msg());
            }
            if (status_speed.valid()) {
                try {
                    _joint_velocities[i] = _servos[i]->parse_joint_speed(status_speed);
                }
                catch (dynamixel::errors::Error& e) {
                    ROS_ERROR_STREAM("Unpack exception while getting  "
                        << _dynamixel_map[_servos[i]->id()] << "'s velocity\n"
                        << e.msg());
                }
            }
            else {
                ROS_WARN_STREAM("Did not receive any data when reading "
                    << _dynamixel_map[_servos[i]->id()] << "'s velocity");
            }
        }
    }

    template <class Protocol>
    void DynamixelHardwareInterface<Protocol>::write_joints()
    {

        for (unsigned int i = 0; i < _servos.size(); i++) {
            // Sending commands only when needed
            if (std::abs(_joint_commands[i] - _prev_commands[i]) < std::numeric_limits<double>::epsilon())
                continue;
            _prev_commands[i] = _joint_commands[i];
            try {
                dynamixel::StatusPacket<Protocol> status;

                double command = _joint_commands[i];

                OperatingMode mode = _c_mode_map[_servos[i]->id()];
                if (OperatingMode::joint == mode) {
                    typename std::unordered_map<id_t, double>::iterator dynamixel_corrections_iterator
                        = _dynamixel_corrections.find(_servos[i]->id());
                    if (dynamixel_corrections_iterator != _dynamixel_corrections.end()) {
                        command += dynamixel_corrections_iterator->second;
                    }

                    ROS_DEBUG_STREAM("Setting position for joint "
                        << _dynamixel_map[_servos[i]->id()] << " to " << command
                        << " rad.");
                    _dynamixel_controller.send(_servos[i]->reg_goal_position_angle(command));
                    _dynamixel_controller.recv(status);
                }
                else if (OperatingMode::wheel == mode) {
                    ROS_DEBUG_STREAM("Setting velocity for joint "
                        << _servos[i]->id() << " to " << command);
                    _dynamixel_controller.send(_servos[i]->reg_moving_speed_angle(command, mode));
                    _dynamixel_controller.recv(status);
                }
            }
            catch (dynamixel::errors::Error& e) {
                ROS_ERROR_STREAM("Caught a Dynamixel exception while sending "
                    << "new commands:\n"
                    << e.msg());
            }
        }

        try {
            _dynamixel_controller.send(dynamixel::instructions::Action<Protocol>(Protocol::broadcast_id));
        }
        catch (dynamixel::errors::Error& e) {
            ROS_ERROR_STREAM("Caught a Dynamixel exception while sending "
                << "new commands:\n"
                << e.msg());
        }
    }

    /** Serach for the requested servos

        Servos that were not requested are ignored and the software complain if
        any required one misses.
    **/
    template <class Protocol>
    void DynamixelHardwareInterface<Protocol>::_find_servos()
    {
        // extract servo IDs from _dynamixel_map
        std::vector<typename Protocol::id_t> ids(_dynamixel_map.size());
        using dm_iter_t = typename std::unordered_map<id_t, std::string>::iterator;
        for (dm_iter_t dm_iter = _dynamixel_map.begin(); dm_iter != _dynamixel_map.end(); ++dm_iter) {
            ids.push_back(dm_iter->first);
        }

        // get the list of available actuators using the vector of IDs
        try {
            // small recv timeout for auto_detect
            _dynamixel_controller.set_recv_timeout(_scan_timeout);
            _dynamixel_controller.open_serial(_usb_serial_interface, _baudrate);
            _servos = dynamixel::auto_detect<Protocol>(_dynamixel_controller, ids);
        }
        catch (dynamixel::errors::Error& e) {
            ROS_FATAL_STREAM("Caught a Dynamixel exception while trying to "
                << "initialise them:\n"
                << e.msg());
            throw e;
        }

        // restore recv timeout
        _dynamixel_controller.set_recv_timeout(_read_timeout);

        // remove servos that are not in the _dynamixel_map (i.e. that are not used)
        using servo = dynamixel::DynamixelHardwareInterface<Protocol>::dynamixel_servo;
        typename std::vector<servo>::iterator servo_it;
        for (servo_it = _servos.begin(); servo_it != _servos.end();) {
            typename std::unordered_map<id_t, std::string>::iterator dynamixel_iterator
                = _dynamixel_map.find((*servo_it)->id());
            // the actuator's name is not in the map
            if (dynamixel_iterator == _dynamixel_map.end())
                servo_it = _servos.erase(servo_it);
            else
                ++servo_it;
        }

        // Check that no actuator was declared by user but not found
        int missing_servos = _dynamixel_map.size() - _servos.size();
        if (missing_servos > 0) {
            ROS_WARN_STREAM(
                missing_servos
                << " servo"
                << (missing_servos > 1 ? "s were" : " was")
                << " declared to the hardware interface but could not be found");
        }
    }

    /** Enable torque output for a joint and send it the relevant settings.

        For now, these settings are only the speed limit.

        @param servo the actuator concerned
        @param mode operating mode of the actuator (e.g. position or velocity,
            in dynamixel speech, joint, wheel, etc.)
    **/
    template <class Protocol>
    void DynamixelHardwareInterface<Protocol>::_enable_and_configure_servo(dynamixel_servo servo, OperatingMode mode)
    {
        try {
            // enable the actuator
            dynamixel::StatusPacket<Protocol> status;
            ROS_DEBUG_STREAM("Enabling servo " << servo->id());
            _dynamixel_controller.send(servo->set_torque_enable(1));
            _dynamixel_controller.recv(status);

            // set max speed for actuators in position mode
            typename std::unordered_map<id_t, double>::iterator dynamixel_max_speed_iterator
                = _dynamixel_max_speed.find(servo->id());
            if (dynamixel_max_speed_iterator != _dynamixel_max_speed.end()) {
                if (OperatingMode::joint == mode) {
                    dynamixel::StatusPacket<Protocol> status;
                    ROS_DEBUG_STREAM("Setting velocity limit of servo "
                        << _dynamixel_map[servo->id()] << " to "
                        << dynamixel_max_speed_iterator->second << " rad/s.");
                    _dynamixel_controller.send(
                        servo->set_moving_speed_angle(
                            dynamixel_max_speed_iterator->second));
                    _dynamixel_controller.recv(status);
                }
                else {
                    ROS_WARN_STREAM("A \"max speed\" was defined for servo "
                        << servo->id() << " but it is currently only supported for "
                        << "servos in position mode. Ignoring the speed limit.");
                }
            }
        }
        catch (dynamixel::errors::Error& e) {
            ROS_ERROR_STREAM("Caught a Dynamixel exception while "
                << "initializing:\n"
                << e.msg());
        }
    }
} // namespace dynamixel

#endif
