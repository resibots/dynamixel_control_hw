/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, University of Colorado, Boulder
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Univ of CO, Boulder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Original Author: Dave Coleman (https://github.com/davetcoleman/ros_control_boilerplate) */


// for runtime_error
#include <stdexcept>

#include <dynamixel_hardware_interface/dynamixel_loop.hpp>

namespace dynamixel
{
    DynamixelLoop::DynamixelLoop(
        ros::NodeHandle& nh,
        boost::shared_ptr<dynamixel::DynamixelHardwareInterface> hardware_interface)
        : _nh(nh), _hardware_interface(hardware_interface)
    {
        // Create the controller manager
        _controller_manager.reset(new controller_manager::ControllerManager(_hardware_interface.get(), _nh));

        // Load rosparams
        ros::NodeHandle local_node_handle(nh, "dynamixel_hw");
        int error = 0;
        error += !local_node_handle.getParam("loop_hz", _loop_hz);
        error += !local_node_handle.getParam("cycle_time_error_threshold", _cycle_time_error_threshold);
        if(error > 0)
        {
            char error_message[] = "could not retrieve one of the required parameters\n\tdynamixel_hw/loop_hz or dynamixel_hw/cycle_time_error_threshold";
            ROS_ERROR_STREAM(error_message);
            throw std::runtime_error(error_message);
        }

        // Get current time for use with first update
        clock_gettime(CLOCK_MONOTONIC, &_last_time);

        // Start timer that will periodically call DynamixelLoop::update
        ros::Duration _desired_update_freq = ros::Duration(1 / _loop_hz);
        _non_realtime_loop = _nh.createTimer(_desired_update_freq, &DynamixelLoop::update, this);
    }

    void DynamixelLoop::update(const ros::TimerEvent& e)
    {
        // Get change in time
        clock_gettime(CLOCK_MONOTONIC, &_current_time);
        _elapsed_time = ros::Duration(
            _current_time.tv_sec - _last_time.tv_sec +
            (_current_time.tv_nsec - _last_time.tv_nsec) / BILLION);
        _last_time = _current_time;

        // Check cycle time for excess delay
        const double cycle_time_error = (_elapsed_time - _desired_update_freq).toSec();
        if (cycle_time_error > _cycle_time_error_threshold)
        {
            ROS_WARN_STREAM("Cycle time exceeded error threshold by: "
            << cycle_time_error << ", cycle time: " << _elapsed_time
            << ", threshold: " << _cycle_time_error_threshold);
        }

        // Input
        // get the hardware's state
        _hardware_interface->read_joints();

        // Control
        // let the controller compute the new command (via the controller manager)
        _controller_manager->update(ros::Time::now(), _elapsed_time);

        // Output
        // send the new command to hardware
        _hardware_interface->write_joints();
    }

}