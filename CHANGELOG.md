# Main changes to the code

In reverse chronological order:

- replace c-style code with std::chrono for time-related code
- replace std::map with std::unordered_map when possible (hypothetical performance gain)
- remove all dependencies to boost
- max speeds are now defined in radians per seconds
- this package now supports both protocols 1 and 2 of the Dynamixels  
  **the new node names are `dynamixel_control_hw_v1` and `dynamixel_control_hw_v2`**
