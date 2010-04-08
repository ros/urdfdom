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

#include "robot_state_publisher/robot_state_publisher.h"
#include <kdl/frames_io.hpp>
#include <tf_conversions/tf_kdl.h>

using namespace std;
using namespace ros;
using namespace KDL;



namespace robot_state_publisher{

RobotStatePublisher::RobotStatePublisher(const Tree& tree)
   :tree_(tree)
{
  // get tf prefix
  NodeHandle n_local("~");
  std::string searched_param;
  n_local.searchParam("tf_prefix", searched_param);
  n_local.param(searched_param, tf_prefix_, std::string());

  // build tree solver
  solver_.reset(new TreeFkSolverPosFull_recursive(tree_));

  // advertise tf message
  NodeHandle n;
  tf_publisher_ = n.advertise<tf::tfMessage>("/tf", 5);

  // get the root segment of the tree
  SegmentMap::const_iterator root = tree.getRootSegment();
  root_ = root->first;
}



bool RobotStatePublisher::publishTransforms(const map<string, double>& joint_positions, const Time& time)
{
  // calculate transforms form root to every segment in tree
  map<string, Frame> link_poses;
  solver_->JntToCart(joint_positions, link_poses);
  if (link_poses.size() < 2){
    ROS_ERROR("Could not compute link poses. The tree or the state is invalid.");
    return false;
  }
  tf_msg_.transforms.resize(link_poses.size());

  // send transforms to tf
  geometry_msgs::TransformStamped trans;
  tf::Transform tf_frame;
  unsigned int i = 0;
  for (map<string, Frame>::const_iterator f=link_poses.begin(); f!=link_poses.end(); f++){
    tf::TransformKDLToTF(f->second, tf_frame);
    trans.header.stamp = time;
    trans.header.frame_id = tf::resolve(tf_prefix_, root_);
    trans.child_frame_id = tf::resolve(tf_prefix_, f->first);
    tf::transformTFToMsg(tf_frame, trans.transform);
    tf_msg_.transforms[i++] = trans;
  }
  tf_publisher_.publish(tf_msg_);

  return true;
}
}
