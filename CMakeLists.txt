cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_control_hw)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  hardware_interface
  controller_manager
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs message_runtime
)

if ("$ENV{RESIBOTS_DIR}" STREQUAL "")
  set(libdynamixel_dir "~")
else()
  set(libdynamixel_dir "$ENV{RESIBOTS_DIR}")
endif()

link_directories("${libdynamixel_dir}/lib")
include_directories(
  ${catkin_INCLUDE_DIRS}
  "${libdynamixel_dir}/include"
  include
)

add_executable(dynamixel_control_hw src/dynamixel_control_hw.cpp src/dynamixel_loop.cpp src/dynamixel_hardware_interface.cpp)
target_link_libraries(dynamixel_control_hw ${catkin_LIBRARIES} dynamixel)
