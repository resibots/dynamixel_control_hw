cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_control_hw)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(LIBDYNAMIXEL "")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  std_msgs
  message_generation
  hardware_interface
  controller_manager
)

catkin_package(CATKIN_DEPENDS
  roscpp
  urdf
  std_msgs
  message_runtime
)

# Searching for libdynamixel
if ("$ENV{RESIBOTS_DIR}" STREQUAL "" AND "$ENV{LIBDYNAMIXEL}" STREQUAL "")
  message("Using default location for libdynamixel : ~")
  set(libdynamixel_dir "~")
elseif (NOT ("$ENV{LIBDYNAMIXEL}" STREQUAL ""))
  message("Using user-defined location for libdynamixel : ${LIBDYNAMIXEL}")
  set(libdynamixel_dir "$ENV{LIBDYNAMIXEL}")
elseif (NOT ("$ENV{RESIBOTS_DIR}" STREQUAL ""))
  message("Using enviromnetal-defined location for libdynamixel : $ENV{RESIBOTS_DIR}")
  set(libdynamixel_dir "$ENV{RESIBOTS_DIR}")
endif()

if (NOT (EXISTS "${libdynamixel_dir}/include/dynamixel/"))
  message(FATAL_ERROR "libdynamixel was not found!")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  "${libdynamixel_dir}/include"
  include
)

add_executable(dynamixel_control_p1 src/dynamixel_control.cpp)
target_compile_definitions(dynamixel_control_p1 PRIVATE PROTOCOL1)
target_link_libraries(dynamixel_control_p1 ${catkin_LIBRARIES})

add_executable(dynamixel_control_p2 src/dynamixel_control.cpp)
target_link_libraries(dynamixel_control_p2 ${catkin_LIBRARIES})

add_library(dynamixel_hw
  src/hardware_interface.cpp
)
target_link_libraries(dynamixel_hw ${catkin_LIBRARIES})

install(TARGETS dynamixel_hw
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES dynamixel_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
