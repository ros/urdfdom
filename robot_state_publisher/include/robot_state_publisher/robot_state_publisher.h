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

#ifndef ROBOT_STATE_PUBLISHER_H
#define ROBOT_STATE_PUBLISHER_H

#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include "robot_state_publisher/treefksolverposfull_recursive.hpp"

namespace robot_state_publisher{

class RobotStatePublisher
{
public:
  RobotStatePublisher(const KDL::Tree& tree);
  ~RobotStatePublisher(){};

  bool publishTransforms(const std::map<std::string, double>& joint_positions, const ros::Time& time);

private:
  ros::NodeHandle n_;
  ros::Publisher tf_publisher_;
  KDL::Tree tree_;
  boost::scoped_ptr<KDL::TreeFkSolverPosFull_recursive> solver_;
  std::string root_;
  std::string tf_prefix_;
  tf::tfMessage tf_msg_;

  class empty_tree_exception: public std::exception{
    virtual const char* what() const throw(){
      return "Tree is empty";}
  } empty_tree_ex;

};



}

#endif
