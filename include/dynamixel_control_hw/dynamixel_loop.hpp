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

#ifndef DYNAMIXEL_LOOP
#define DYNAMIXEL_LOOP

#include <chrono>

// for shared pointer
#include <memory>
// for runtime_error
#include <stdexcept>

// ROS
#include <ros/ros.h>

// ROS control
#include <controller_manager/controller_manager.h>

// The hardware interface to dynamixels
#include <dynamixel_control_hw/dynamixel_hardware_interface.hpp>

namespace dynamixel {
    // To make use of steady_clock and duration_cast shorter
    using namespace std::chrono;

    template <class Protocol>
    class DynamixelLoop {
        using hw_int = typename dynamixel::DynamixelHardwareInterface<Protocol>;

    public:
        DynamixelLoop(
            ros::NodeHandle& nh,
            std::shared_ptr<hw_int> hardware_interface)
            : _nh(nh),
              _hardware_interface(hardware_interface)
        {
            // Create the controller manager
            _controller_manager.reset(new controller_manager::ControllerManager(_hardware_interface.get(), _nh));

            // Load rosparams
            int error = 0;
            ros::NodeHandle np("~");
            error += !np.getParam("loop_frequency", _loop_hz);
            error += !np.getParam("cycle_time_error_threshold", _cycle_time_error_threshold);
            if (error > 0) {
                char error_message[] = "could not retrieve one of the required parameters\n\tdynamixel_hw/loop_hz or dynamixel_hw/cycle_time_error_threshold";
                ROS_ERROR_STREAM(error_message);
                throw std::runtime_error(error_message);
            }

            // Get current time for use with first update
            _last_time = steady_clock::now();

            // Start timer that will periodically call DynamixelLoop::update
            ros::Duration _desired_update_freq = ros::Duration(1 / _loop_hz);
            _non_realtime_loop = _nh.createTimer(_desired_update_freq, &DynamixelLoop::update, this);
        }

        /** Timed method that reads current hardware's state, runs the controller
            code once and sends the new commands to the hardware.

            Note: we do not use the TimerEvent time difference because it
                does NOT guarantee that the time source is strictly
                linearly increasing.
        **/
        void update(const ros::TimerEvent&)
        {
            // Get change in time
            _current_time = steady_clock::now();
            duration<double> time_span
                = duration_cast<duration<double>>(_current_time - _last_time);
            _elapsed_time = ros::Duration(time_span.count());
            _last_time = _current_time;

            // Check cycle time for excess delay
            const double cycle_time_error = (_elapsed_time - _desired_update_freq).toSec();
            if (cycle_time_error > _cycle_time_error_threshold) {
                ROS_WARN_STREAM("Cycle time exceeded error threshold by: "
                    << cycle_time_error - _cycle_time_error_threshold << "s, "
                    << "cycle time: " << _elapsed_time << "s, "
                    << "threshold: " << _cycle_time_error_threshold << "s");
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

    private:
        // Startup and shutdown of the internal node inside a roscpp program
        ros::NodeHandle _nh;

        // Settings
        ros::Duration _desired_update_freq;
        double _cycle_time_error_threshold;

        // Timing
        ros::Timer _non_realtime_loop;
        ros::Duration _elapsed_time;
        double _loop_hz;
        steady_clock::time_point _last_time;
        steady_clock::time_point _current_time;

        /** ROS Controller Manager and Runner

            This class advertises a ROS interface for loading, unloading, starting, and
            stopping ros_control-based controllers. It also serializes execution of all
            running controllers in \ref update.
        **/
        std::shared_ptr<controller_manager::ControllerManager> _controller_manager;

        /// Abstract Hardware Interface for your robot
        std::shared_ptr<hw_int> _hardware_interface;
    };
} // namespace dynamixel

#endif
