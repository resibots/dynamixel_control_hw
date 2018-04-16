#include <pluginlib/class_list_macros.h>
#include <dynamixel_control_hw/dynamixel_hardware_interface.hpp>

namespace dynamixel {
    typedef dynamixel::DynamixelHardwareInterface<Protocol1> DynamixelHardwareInterfaceP1;
    typedef dynamixel::DynamixelHardwareInterface<Protocol2> DynamixelHardwareInterfaceP2;
}

PLUGINLIB_EXPORT_CLASS(dynamixel::DynamixelHardwareInterfaceP1, hardware_interface::RobotHW)
PLUGINLIB_EXPORT_CLASS(dynamixel::DynamixelHardwareInterfaceP2, hardware_interface::RobotHW)