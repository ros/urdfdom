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
#include <kdl_parser/dom_parser.hpp>
#include <urdf/model.h>
#include "robot_state_publisher/joint_state_listener.h"

using namespace std;
using namespace ros;
using namespace KDL;
using namespace robot_state_publisher;



// ----------------------------------
// ----- MAIN -----------------------
// ----------------------------------
int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, "robot_state_publisher");
  NodeHandle node;

  // gets the location of the robot description on the parameter server
  string param_name, full_param_name;
  node.param("~/robot_desc_param", param_name, string("robot_description"));
  node.searchParam(param_name,full_param_name);
  string robot_desc;

  // constructs a kdl tree from the robot model
  node.param(full_param_name, robot_desc, string());
  Tree tree;
  if (!kdl_parser::treeFromString(robot_desc, tree)){
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return -1;
  }

  JointStateListener state_publisher(tree);

  ros::spin();
  return 0;
}
