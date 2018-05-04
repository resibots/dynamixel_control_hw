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

// Local includes
#include <dynamixel_control/combined_control_loop.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "combined_robot_control");
    ros::NodeHandle nh;
    ros::NodeHandle robot_hw_nh("~");

    // We run the ROS loop in a separate thread as external calls, such
    // as service callbacks loading controllers, can block the (main) control loop
    ros::AsyncSpinner spinner(2);
    spinner.start();

    try {
        // Start the control loop
        combined_hw::CombinedRobotLoop control_loop(nh, robot_hw_nh);

        // Wait until shutdown signal received
        ros::waitForShutdown();
    }
    catch (const ros::Exception& e) {
        ROS_FATAL_STREAM("Error in the hardware interface:\n"
            << "\t" << e.what());
        return 1;
    }

    return 0;
}
