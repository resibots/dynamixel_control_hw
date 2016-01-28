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

// Standard C++
#include <time.h>

// Boost shared pointer
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>

// ROS control
#include <controller_manager/controller_manager.h>

// The hardware interface to dynamixels
#include <dynamixel_control_hw/dynamixel_hardware_interface.hpp>

namespace dynamixel
{
    // Used to convert seconds elapsed to nanoseconds
    static const double BILLION = 1000000000.0;

    class DynamixelLoop
    {
    public:
        DynamixelLoop(ros::NodeHandle& nh, boost::shared_ptr<dynamixel::DynamixelHardwareInterface> hardware_interface);

        /** Timed method that reads current hardware's state, runs the controller
            code once and sends the new commands to the hardware.

            Note: we do not use the TimerEvent time difference because it
                does NOT guarantee that the time source is strictly
                linearly increasing.
        **/
        void update(const ros::TimerEvent& e);
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
        struct timespec _last_time;
        struct timespec _current_time;

        /** ROS Controller Manager and Runner

        This class advertises a ROS interface for loading, unloading, starting, and
        stopping ros_control-based controllers. It also serializes execution of all
        running controllers in \ref update.
        **/
        boost::shared_ptr<controller_manager::ControllerManager> _controller_manager;

        /// Abstract Hardware Interface for your robot
        boost::shared_ptr<dynamixel::DynamixelHardwareInterface> _hardware_interface;
    };
}

# endif