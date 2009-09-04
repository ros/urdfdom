/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Wim Meeussen */

#include <kdl/tree.hpp>
#include <ros/ros.h>
#include "robot_state_publisher/robot_state_publisher.h"
#include "robot_state_publisher/joint_state_listener.h"

using namespace std;
using namespace ros;
using namespace KDL;
using namespace robot_state_publisher;


JointStateListener::JointStateListener(const KDL::Tree& tree)
  : publish_rate_(0.0), state_publisher_(tree)
{
  // set publish frequency
  double publish_freq;
  n_.param("~publish_frequency", publish_freq, 50.0);
  publish_rate_ = Rate(publish_freq);
  
  // subscribe to mechanism state
  string joint_state_topic;
  n_.param("~joint_state_topic", joint_state_topic, string("joint_states"));
  joint_state_sub_ = n_.subscribe(joint_state_topic, 1, &JointStateListener::callbackJointState, this);;
};


JointStateListener::~JointStateListener()
{};


void JointStateListener::callbackJointState(const JointStateConstPtr& state)
{
  if (state->get_name_size() == 0){
    ROS_ERROR("Robot state publisher received an empty joint state vector");
    return;
  }

  if (state->get_name_size() != state->get_position_size()){
    ROS_ERROR("Robot state publisher received an invalid joint state vector");
    return;
  }

  // get joint positions from state message
  map<string, double> joint_positions;
  for (unsigned int i=0; i<state->name.size(); i++)
    joint_positions.insert(make_pair(state->name[i], state->position[i]));
  state_publisher_.publishTransforms(joint_positions, state->header.stamp);
  publish_rate_.sleep();
}

