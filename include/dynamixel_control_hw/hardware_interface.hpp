#ifndef dynamixel_hardware_interface
#define dynamixel_hardware_interface

#include <limits>
#include <math.h>
#include <stdexcept>
#include <unordered_map>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>
// ROS-related: to parse parameters
#include <XmlRpcException.h>
#include <XmlRpcValue.h>

// ROS control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

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

        DynamixelHardwareInterface(){};
        ~DynamixelHardwareInterface();

        /** Initialise the whole hardware interface.

            Set the essential parameters for communication with the hardware
            and find all connected devices and register those referred in
            dynamixel_map in the hardware interface.
        **/
        bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);

        /** Copy joint's information to memory

            firstly queries the information from the dynamixels, then put it in
            private attributes, for use by a controller.

            Warning: do not get any information on torque
        **/
        virtual void read(const ros::Time& time, const ros::Duration& elapsed_time);

        /** Send new joint's target position to dynamixels

            takes the target position from memory (given by a controller) and sends
            them to the dynamixels.
        **/
        virtual void write(const ros::Time& time, const ros::Duration& elapsed_time);

    private:
        using dynamixel_servo = std::shared_ptr<dynamixel::servos::BaseServo<Protocol>>;

        DynamixelHardwareInterface(DynamixelHardwareInterface<Protocol> const&) = delete;
        DynamixelHardwareInterface& operator=(DynamixelHardwareInterface<Protocol> const&) = delete;

        // Methods used for initialisation
        bool _get_ros_parameters(ros::NodeHandle& root_nh,
            ros::NodeHandle& robot_hw_nh);
        dynamixel::OperatingMode _str2mode(const std::string& mode_string,
            std::string name);
        bool _load_urdf(ros::NodeHandle& nh, std::string param_name);
        bool _find_servos();
        void _enable_and_configure_servo(dynamixel_servo servo, OperatingMode mode);
        void _register_joint_limits(const hardware_interface::JointHandle& cmd_handle,
            id_t id);

        void _enforce_limits(const ros::Duration& loop_period);

        // ROS's hardware interface instances
        hardware_interface::JointStateInterface _jnt_state_interface;
        hardware_interface::PositionJointInterface _jnt_pos_interface;
        hardware_interface::VelocityJointInterface _jnt_vel_interface;

        // Joint limits (hard and soft)
        joint_limits_interface::PositionJointSoftLimitsInterface _jnt_pos_lim_interface;
        joint_limits_interface::VelocityJointSoftLimitsInterface _jnt_vel_lim_interface;
        joint_limits_interface::PositionJointSaturationInterface _jnt_pos_sat_interface;
        joint_limits_interface::VelocityJointSaturationInterface _jnt_vel_sat_interface;

        // Memory space shared with the controller
        // It reads here the latest robot's state and put here the next desired values
        std::vector<double> _prev_commands;
        std::vector<double> _joint_commands; // target joint angle or speed
        std::vector<double> _joint_angles; // actual joint angle
        std::vector<double> _joint_velocities; // actual joint velocity
        std::vector<double> _joint_efforts; // compulsory but not used

        // USB to serial connection settings
        std::string _usb_serial_interface;
        int _baudrate;
        float _read_timeout, _scan_timeout; // in seconds
        // Dynamixel's low level controller
        dynamixel::controllers::Usb2Dynamixel _dynamixel_controller;

        // List of actuators (collected at init. from the actuators)
        std::vector<dynamixel_servo> _servos;
        // Map from dynamixel ID to actuator's name
        std::unordered_map<id_t, std::string> _dynamixel_map;
        // Map from dynamixel ID to actuator's command interface (ID: velocity/position)
        std::unordered_map<id_t, OperatingMode> _c_mode_map;
        // Map for servos in velocity mode which speed needs to be inverted
        std::unordered_map<id_t, bool> _invert;
        // Map for max speed (ID: max speed)
        std::unordered_map<id_t, double> _dynamixel_max_speed;
        // Map for angle offsets (ID: correction in radians)
        std::unordered_map<id_t, double> _dynamixel_corrections;

        // To get joint limits from the parameter server
        ros::NodeHandle _nh;

        // URDF model of the robot, for joint limits
        std::shared_ptr<urdf::Model> _urdf_model;
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
            ROS_ERROR_STREAM("Caught a Dynamixel exception while trying to "
                << "power them off:\n"
                << e.msg());
        }
    }

    template <class Protocol>
    bool DynamixelHardwareInterface<Protocol>::init(
        ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
    {
        // Get the relevant parameters from rosparam
        // Search for the servos declared bu the user, in the parameters
        if (!_get_ros_parameters(root_nh, robot_hw_nh) || !_find_servos())
            return false;

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
                    else if (OperatingMode::unknown != hardware_mode) {
                        ROS_ERROR_STREAM("Servo " << id << " was not initialised "
                                                  << "(operating mode "
                                                  << mode2str(hardware_mode)
                                                  << " is not supported)");
                        _c_mode_map[id] = OperatingMode::unknown;
                    }

                    // Enable servos that were properly configured
                    if (OperatingMode::unknown != _c_mode_map[id]) {
                        // Set joint limits (saturation or soft for the joint)
                        _register_joint_limits(cmd_handle, id);
                        // enable torque output on the servo and set its configuration
                        // including max speed
                        _enable_and_configure_servo(_servos[i], hardware_mode);
                    }
                    else {
                        // remove this servo
                        _servos.erase(_servos.begin() + i);
                        --i;
                    }
                }
                else {
                    ROS_WARN_STREAM("Servo " << id << " was not initialised "
                                             << "(not found in the parameters)");
                    // remove this servo
                    _servos.erase(_servos.begin() + i);
                    --i;
                }
            }

            // register the hardware interfaces
            registerInterface(&_jnt_state_interface);
            registerInterface(&_jnt_pos_interface);
            registerInterface(&_jnt_vel_interface);
        }
        catch (const ros::Exception& e) {
            // TODO: disable actuators that were enabled ?
            ROS_FATAL_STREAM("Error during initialisation. BEWARE: some "
                << "actuators might have already been started.");
            return false;
        }

        // At startup robot should keep the pose it has
        ros::Duration period(0);
        read(ros::Time::now(), period);

        for (unsigned i = 0; i < _servos.size(); i++) {
            OperatingMode mode = _c_mode_map[_servos[i]->id()];
            if (OperatingMode::joint == mode)
                _joint_commands[i] = _joint_angles[i];
            else if (OperatingMode::wheel == mode)
                _joint_commands[i] = 0;
        }

        return true;
    }

    template <class Protocol>
    void DynamixelHardwareInterface<Protocol>::read(const ros::Time& time,
        const ros::Duration& elapsed_time)
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

                // Invert the orientation, if configured
                typename std::unordered_map<id_t, bool>::iterator
                    invert_iterator
                    = _invert.find(_servos[i]->id());
                if (invert_iterator != _invert.end()) {
                    _joint_angles[i] = 2 * M_PI - _joint_angles[i];
                }

                // Apply angle correction to joint, if any
                typename std::unordered_map<id_t, double>::iterator
                    dynamixel_corrections_iterator
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
                    _joint_velocities[i]
                        = _servos[i]->parse_joint_speed(status_speed);

                    typename std::unordered_map<id_t, bool>::iterator
                        invert_iterator
                        = _invert.find(_servos[i]->id());
                    if (invert_iterator != _invert.end()
                        && invert_iterator->second)
                        _joint_velocities[i] = -_joint_velocities[i];
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
    void DynamixelHardwareInterface<Protocol>::write(const ros::Time& time,
        const ros::Duration& loop_period)
    {
        // ensure that the joints limits are respected
        _enforce_limits(loop_period);

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
                    typename std::unordered_map<id_t, double>::iterator
                        dynamixel_corrections_iterator
                        = _dynamixel_corrections.find(_servos[i]->id());
                    if (dynamixel_corrections_iterator != _dynamixel_corrections.end()) {
                        command += dynamixel_corrections_iterator->second;
                    }

                    // Invert the orientation, if configured
                    typename std::unordered_map<id_t, bool>::iterator
                        invert_iterator
                        = _invert.find(_servos[i]->id());
                    if (invert_iterator != _invert.end()) {
                        command = 2 * M_PI - command;
                    }

                    ROS_DEBUG_STREAM("Setting position for joint "
                        << _dynamixel_map[_servos[i]->id()] << " to " << command
                        << " rad.");
                    _dynamixel_controller.send(
                        _servos[i]->reg_goal_position_angle(command));
                    _dynamixel_controller.recv(status);
                }
                else if (OperatingMode::wheel == mode) {
                    // Invert the orientation, if configured
                    const short sign = 1; // formerly: _invert[_servos[i]->id()] ? -1 : 1;
                    typename std::unordered_map<id_t, bool>::iterator
                        invert_iterator
                        = _invert.find(_servos[i]->id());
                    if (invert_iterator != _invert.end()
                        && invert_iterator->second) {
                        command = -command;
                    }
                    ROS_DEBUG_STREAM("Setting velocity for joint "
                        << _servos[i]->id() << " to " << command);
                    _dynamixel_controller.send(
                        _servos[i]->reg_moving_speed_angle(command, mode));
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

    /** Retrieve all needed ROS parameters
        @return true if and only if all went well
    **/
    template <class Protocol>
    bool DynamixelHardwareInterface<Protocol>::_get_ros_parameters(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
    {
        bool got_all_params = true;

        got_all_params &= robot_hw_nh.getParam("serial_interface", _usb_serial_interface); // path to the device
        int baudrate;
        got_all_params &= robot_hw_nh.getParam("baudrate", baudrate); // in bauds
        _baudrate = get_baudrate(baudrate); // convert to OS-defined values
        got_all_params &= robot_hw_nh.getParam("read_timeout", _read_timeout); // in seconds
        bool dyn_scan = robot_hw_nh.getParam("scan_timeout", _scan_timeout); // in seconds
        if (!dyn_scan) {
            ROS_WARN_STREAM("Dynamixel scanning timeout parameter was not found. "
                << "Setting to default: 0.05s.");
            _scan_timeout = 0.05;
        }
        std::string default_command_interface;
        bool has_default_command_interface = robot_hw_nh.getParam(
            "default_command_interface", default_command_interface);

        // Retrieve the servo parameter and fill maps above with its content.
        XmlRpc::XmlRpcValue servos_param; // temporary map, from parameter server
        if (got_all_params &= robot_hw_nh.getParam("servos", servos_param)) {
            ROS_ASSERT(servos_param.getType() == XmlRpc::XmlRpcValue::TypeStruct);
            try {
                for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = servos_param.begin(); it != servos_param.end(); ++it) {
                    ROS_DEBUG_STREAM("servo: " << (std::string)(it->first));

                    id_t id;
                    if (it->second.hasMember("id")) {
                        id = static_cast<int>(servos_param[it->first]["id"]);
                        ROS_DEBUG_STREAM("\tid: " << (int)id);
                        _dynamixel_map[id] = it->first;
                    }
                    else {
                        ROS_ERROR_STREAM("Actuator " << it->first
                                                     << " has no associated servo ID.");
                        continue;
                    }

                    if (it->second.hasMember("offset")) {
                        _dynamixel_corrections[id]
                            = static_cast<double>(servos_param[it->first]["offset"]);
                        ROS_DEBUG_STREAM("\toffset: "
                            << _dynamixel_corrections[id]);
                    }

                    if (it->second.hasMember("command_interface")) {
                        std::string mode_string
                            = static_cast<std::string>(
                                servos_param[it->first]["command_interface"]);
                        ROS_DEBUG_STREAM("\tcommand_interface: " << mode_string);

                        _c_mode_map[id] = _str2mode(mode_string, it->first);
                    }
                    else if (has_default_command_interface) {
                        _c_mode_map[id]
                            = _str2mode(default_command_interface, it->first);
                        ROS_DEBUG_STREAM("\tcommand_interface: "
                            << default_command_interface << " (default)");
                    }
                    else {
                        ROS_ERROR_STREAM("A command interface (speed or position) "
                            << "should be declared for the actuator " << it->first
                            << " or a default one should be defined with the parameter "
                            << "'default_command_interface'.");
                    }

                    if (it->second.hasMember("invert_velocity")) {
                        _invert[id] = servos_param[it->first]["invert_velocity"];
                    }
                }
            }
            catch (XmlRpc::XmlRpcException& e) {
                ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
                    << "configuration: " << e.getMessage() << ".\n"
                    << "Please check the configuration, particularly parameter types.");
                return false;
            }
        }

        if (!got_all_params) {
            std::string sub_namespace = robot_hw_nh.getNamespace();
            std::string error_message = "One or more of the following parameters "
                                        "were not set:\n"
                + sub_namespace + "/serial_interface\n"
                + sub_namespace + "/baudrate\n"
                + sub_namespace + "/read_timeout\n"
                + sub_namespace + "/servos";
            ROS_FATAL_STREAM(error_message);
            return false;
        }

        // Get joint limits from the URDF or the parameter server
        // ------------------------------------------------------

        std::string urdf_param_name("/robot_description");
        // TODO: document this feature
        if (robot_hw_nh.hasParam("urdf_param_name"))
            robot_hw_nh.getParam("urdf_param_name", urdf_param_name);

        if (!_load_urdf(root_nh, urdf_param_name))
            ROS_INFO_STREAM("Unable to find a URDF model.");
        else
            ROS_DEBUG_STREAM("Received the URDF from param server.");

        return true;
    }

    /** Convert a string to an operating mode for a Dynamixel servo

        @param mode_string name of the operating mode (either velocity or position)
        @param nam name of the joint
        @return dynamixel::OperatingMode::unknown if mode_string is not recognized
    **/
    template <class Protocol>
    dynamixel::OperatingMode DynamixelHardwareInterface<Protocol>::_str2mode(
        const std::string& mode_string, std::string name)
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

    /** Search for the robot's URDF model on the parameter server and parse it

        @param nh NodeHandle used to query for the URDF
        @param param_name name of the ROS parameter holding the robot model
        @param urdf_model pointer to be populated by this function
    **/
    template <class Protocol>
    bool DynamixelHardwareInterface<Protocol>::_load_urdf(ros::NodeHandle& nh,
        std::string param_name)
    {
        std::string urdf_string;
        if (_urdf_model == nullptr)
            _urdf_model = std::make_shared<urdf::Model>();

        // get the urdf param on param server
        nh.getParam(param_name, urdf_string);

        return !urdf_string.empty() && _urdf_model->initString(urdf_string);
    }

    /** Search for the requested servos

        Servos that were not requested are ignored and the software complain if
        any required one misses.

        @return true if and only if there was no error
    **/
    template <class Protocol>
    bool DynamixelHardwareInterface<Protocol>::_find_servos()
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
            // restore recv timeout
            _dynamixel_controller.set_recv_timeout(_read_timeout);
        }
        catch (dynamixel::errors::Error& e) {
            ROS_FATAL_STREAM("Caught a Dynamixel exception while trying to "
                << "initialise them:\n"
                << e.msg());
            return false;
        }

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
            ROS_ERROR_STREAM(
                missing_servos
                << " servo"
                << (missing_servos > 1 ? "s were" : " was")
                << " declared to the hardware interface but could not be found");
            return false;
        }

        _prev_commands.resize(_servos.size(), 0.0);
        _joint_commands.resize(_servos.size(), 0.0);
        _joint_angles.resize(_servos.size(), 0.0);
        _joint_velocities.resize(_servos.size(), 0.0);
        _joint_efforts.resize(_servos.size(), 0.0);

        return true;
    }

    /** Enable torque output for a joint and send it the relevant settings.

        For now, these settings are only the speed limit.

        @param servo the actuator concerned
        @param mode operating mode of the actuator (e.g. position or velocity,
            in dynamixel speech, joint, wheel, etc.)
    **/
    template <class Protocol>
    void DynamixelHardwareInterface<Protocol>::_enable_and_configure_servo(
        dynamixel_servo servo, OperatingMode mode)
    {
        try {
            // Enable the actuator

            dynamixel::StatusPacket<Protocol> status;
            ROS_DEBUG_STREAM("Enabling servo " << servo->id());
            _dynamixel_controller.send(servo->set_torque_enable(1));
            _dynamixel_controller.recv(status);

            // Set max speed for actuators in position mode

            typename std::unordered_map<id_t, double>::iterator
                dynamixel_max_speed_iterator
                = _dynamixel_max_speed.find(servo->id());
            if (dynamixel_max_speed_iterator != _dynamixel_max_speed.end()) {
                if (OperatingMode::joint == mode) {
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
            else if (OperatingMode::joint == mode) {
                ROS_DEBUG_STREAM("Resetting velocity limit of servo "
                    << _dynamixel_map[servo->id()] << ".");
                _dynamixel_controller.send(servo->set_moving_speed_angle(0));
                _dynamixel_controller.recv(status);
            }
        }
        catch (dynamixel::errors::Error& e) {
            ROS_ERROR_STREAM("Caught a Dynamixel exception while "
                << "initializing:\n"
                << e.msg());
        }
    }

    template <class Protocol>
    void DynamixelHardwareInterface<Protocol>::_register_joint_limits(
        const hardware_interface::JointHandle& cmd_handle,
        id_t id)
    {
        // Limits datastructures
        joint_limits_interface::JointLimits joint_limits; // Position
        joint_limits_interface::SoftJointLimits soft_limits; // Soft Position
        bool has_joint_limits = false;
        bool has_soft_limits = false;

        // Get limits from URDF
        if (_urdf_model == NULL) {
            ROS_WARN_STREAM("No URDF model loaded, cannot be used to load joint"
                            " limits");
        }
        else {
            // Get limits from URDF
            urdf::JointConstSharedPtr urdf_joint
                = _urdf_model->getJoint(cmd_handle.getName());

            // Get main joint limits
            if (urdf_joint == nullptr) {
                ROS_ERROR_STREAM("URDF joint not found "
                    << cmd_handle.getName());
                return;
            }

            // Get limits from URDF
            if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits)) {
                has_joint_limits = true;
                ROS_DEBUG_STREAM("Joint " << cmd_handle.getName()
                                          << " has URDF position limits ["
                                          << joint_limits.min_position << ", "
                                          << joint_limits.max_position << "]");
                if (joint_limits.has_velocity_limits)
                    ROS_DEBUG_STREAM("Joint " << cmd_handle.getName()
                                              << " has URDF velocity limit ["
                                              << joint_limits.max_velocity << "]");
            }
            else {
                if (urdf_joint->type != urdf::Joint::CONTINUOUS)
                    ROS_WARN_STREAM("Joint " << cmd_handle.getName()
                                             << " does not have a URDF "
                                                "position limit");
            }

            // Get soft limits from URDF
            if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits)) {
                has_soft_limits = true;
                ROS_DEBUG_STREAM("Joint " << cmd_handle.getName()
                                          << " has soft joint limits.");
            }
            else {
                ROS_DEBUG_STREAM("Joint " << cmd_handle.getName()
                                          << " does not have soft joint "
                                             "limits");
            }
        }

        // Get limits from ROS param
        if (joint_limits_interface::getJointLimits(cmd_handle.getName(), _nh, joint_limits)) {
            has_joint_limits = true;
            ROS_DEBUG_STREAM("Joint " << cmd_handle.getName()
                                      << " has rosparam position limits ["
                                      << joint_limits.min_position << ", "
                                      << joint_limits.max_position << "]");
            if (joint_limits.has_velocity_limits)
                ROS_DEBUG_STREAM("Joint " << cmd_handle.getName()
                                          << " has rosparam velocity limit ["
                                          << joint_limits.max_velocity << "]");
        } // the else debug message provided internally by joint_limits_interface

        // Slightly reduce the joint limits to prevent floating point errors
        if (joint_limits.has_position_limits) {
            joint_limits.min_position += std::numeric_limits<double>::epsilon();
            joint_limits.max_position -= std::numeric_limits<double>::epsilon();
        }

        // Save the velocity limit for later if the joint is in position mode
        // it is going to be sent to the servo-motor which will enforce it.
        if (joint_limits.has_velocity_limits
            && OperatingMode::joint == _c_mode_map[id]) {
            _dynamixel_max_speed[id] = joint_limits.max_velocity;
        }

        if (has_soft_limits) // Use soft limits
        {
            ROS_DEBUG_STREAM("Using soft saturation limits");

            if (OperatingMode::joint == _c_mode_map[id]) {
                const joint_limits_interface::PositionJointSoftLimitsHandle
                    soft_handle_position(
                        cmd_handle, joint_limits, soft_limits);
                _jnt_pos_lim_interface.registerHandle(soft_handle_position);
            }
            else if (OperatingMode::wheel == _c_mode_map[id]) {
                const joint_limits_interface::VelocityJointSoftLimitsHandle
                    soft_handle_velocity(
                        cmd_handle, joint_limits, soft_limits);
                _jnt_vel_lim_interface.registerHandle(soft_handle_velocity);
            }
        }
        else if (has_joint_limits) // Use saturation limits
        {
            ROS_DEBUG_STREAM("Using saturation limits (not soft limits)");
            if (OperatingMode::joint == _c_mode_map[id]) {
                const joint_limits_interface::PositionJointSaturationHandle
                    sat_handle_position(cmd_handle, joint_limits);
                _jnt_pos_sat_interface.registerHandle(sat_handle_position);
            }
            else if (OperatingMode::wheel == _c_mode_map[id]) {
                const joint_limits_interface::VelocityJointSaturationHandle
                    sat_handle_velocity(cmd_handle, joint_limits);
                _jnt_vel_sat_interface.registerHandle(sat_handle_velocity);
            }
        }
    } // namespace dynamixel

    template <class Protocol>
    void DynamixelHardwareInterface<Protocol>::_enforce_limits(const ros::Duration& loop_period)
    {
        // enforce joint limits
        _jnt_pos_lim_interface.enforceLimits(loop_period);
        _jnt_vel_lim_interface.enforceLimits(loop_period);
        _jnt_pos_sat_interface.enforceLimits(loop_period);
        _jnt_vel_sat_interface.enforceLimits(loop_period);
    }
} // namespace dynamixel

#endif
