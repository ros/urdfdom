// Copyright  (C)  2009 Willow Garage Inc

// Version: 1.0
// Author: Wim Meeussen <meeussen at willowgarage dot com>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


#include "robot_state_publisher/treefksolverposfull_recursive.hpp"
#include <iostream>

using namespace std;

namespace KDL {

TreeFkSolverPosFull_recursive::TreeFkSolverPosFull_recursive(const Tree& _tree):
  tree(_tree)
{
}

TreeFkSolverPosFull_recursive::~TreeFkSolverPosFull_recursive()
{
}

int TreeFkSolverPosFull_recursive::JntToCart(const map<string, double>& q_in, map<string, Frame>& p_out)
{      
  // clear output
  p_out.clear();

  addFrameToMap(q_in, p_out, Frame::Identity(), tree.getRootSegment());

  return 0;
}



void TreeFkSolverPosFull_recursive::addFrameToMap(const map<string, double>& q_in, map<string, Frame>& p_out,
                                                  const Frame& previous_frame, const SegmentMap::const_iterator this_segment)
{
  // get pose of this segment
  Frame this_frame = previous_frame;
  double jnt_p = 0;
  if (this_segment->second.segment.getJoint().getType() != Joint::None){
    map<string, double>::const_iterator jnt_pos = q_in.find(this_segment->second.segment.getJoint().getName());
    if (jnt_pos == q_in.end()){
      printf("Could not find value for joint %s\n", this_segment->first.c_str());
      return;
    }
    jnt_p = jnt_pos->second;
  }
  this_frame = this_frame * this_segment->second.segment.pose(jnt_p);
  if (this_segment->first != tree.getRootSegment()->first)
    p_out.insert(make_pair(this_segment->first, this_frame));

  // get poses of child segments
  for (vector<SegmentMap::const_iterator>::const_iterator child=this_segment->second.children.begin(); child !=this_segment->second.children.end(); child++)
    addFrameToMap(q_in, p_out, this_frame, *child);

}


}
