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

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the Scorbot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <scorbot_control/scorbot_hw_interface.h>

//includes of the joint state and command services
#include <ros_control_boilerplate/read_joint_states.h>
#include <ros_control_boilerplate/write_joint_cmd.h>


namespace scorbot_control
{

  //ros::ServiceClient client = nh.serviceClient<ros_control_boilerplate::read_joint_states>("read_joint_states");
  ros_control_boilerplate::read_joint_states srv_read;

  // ros::ServiceClient client = nh.serviceClient<ros_control_boilerplate::write_joint_cmd>("write_joint_cmd");
  // ros_control_boilerplate::write_joint_cmd srv_write;
ScorbotHWInterface::ScorbotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  ROS_INFO_NAMED("scorbot_hw_interface", "ScorbotHWInterface Ready.");
}



void ScorbotHWInterface::read(ros::Duration &elapsed_time)
{
  srv_read.request.input = float(1234); //sending a random input value as request
  if (ros::service::call("read_joint_states",srv_read))
  {
    //The recieved values are  joint0, joint1, ......
    joint_position_[0] = srv_read.response.joint0;
    joint_position_[1] = srv_read.response.joint1;
    joint_position_[2] = srv_read.response.joint2;
    joint_position_[3] = srv_read.response.joint3;
    joint_position_[4] = srv_read.response.joint4;
  }
  else
  {
    ROS_ERROR("Failed to call service read_joint_states");
  }
}

void ScorbotHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR WRITE COMMAND TO USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  
  // srv_write.request.cmd0 = joint_position_command_[0]; //sending the commands
  // srv_write.request.cmd1 = joint_position_command_[1];
  // srv_write.request.cmd2 = joint_position_command_[2];
  // srv_write.request.cmd3 = joint_position_command_[3];
  // srv_write.request.cmd4 = joint_position_command_[4];

  // if (client.call(srv_write))
  // {
  //   // status recieved at srv_write.input.status;
  // }
  // else
  // {
  //   ROS_ERROR("Failed to call service read_joint_states");
  //}
  // FOR A EASY SIMULATION EXAMPLE, OR FOR CODE TO CALCULATE
  // VELOCITY FROM POSITION WITH SMOOTHING, SEE
  // sim_hw_interface.cpp IN THIS PACKAGE
  

  // DUMMY PASS-THROUGH CODE
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    joint_position_[joint_id] += joint_position_command_[joint_id];
  // END DUMMY CODE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void ScorbotHWInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

}  // namespace
