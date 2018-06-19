# Main changes to the code

In reverse chronological order:

- allow an actuator to be inverted (angle and speed)
- send velocity limits to the actuators, if they are in position mode
- rename the node executable so that the suffixes reflect the protocl version
- make plugins for this hardware interface to be included in [combined hardware interfaces](http://wiki.ros.org/combined_robot_hw)
- implement joint limits (parameterised through URDF or param server)
- breaking change: the configuration parameter syntax has changed; see sample configuration files
- velocity control mode integrated
- replace c-style code with std::chrono for time-related code
- replace std::map with std::unordered_map when possible (hypothetical performance gain)
- remove all dependencies to boost
- max speeds are now defined in radians per seconds
- this package now supports both protocols 1 and 2 of the Dynamixels  
  **the new node names are `dynamixel_control_hw_v1` and `dynamixel_control_hw_v2`**
