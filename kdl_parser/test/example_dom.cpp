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

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include "kdl_parser/dom_parser.hpp"
#include <iostream>

using namespace KDL;
using namespace std;
using namespace urdf;

int main()
{
  Model robot_model;
  if (!robot_model.initFile("pr2.urdf"))
  {cerr << "Could not generate robot model" << endl; return false;}

  Tree my_tree;
  if (!kdl_parser::treeFromRobotModel(robot_model, my_tree)) 
  {cerr << "Could not extract kdl tree" << endl; return false;}

  // walk through tree
  cout << " ======================================" << endl;
  SegmentMap::const_iterator root = my_tree.getRootSegment();
  cout << "Found root segment '" << root->second.segment.getName() << "' with " << root->second.children.size() << " children" << endl;
  for (unsigned int i=0; i<root->second.children.size(); i++){
    SegmentMap::const_iterator child = root->second.children[i];
    cout << " - child " << i+1 << ": " << child->second.segment.getName() << " has joint " << child->second.segment.getJoint().getName() 
         << " and " << child->second.children.size() << " children" << endl;

    for (unsigned int j=0; j<child->second.children.size(); j++){
      SegmentMap::const_iterator grandchild = child->second.children[j];
      cout << "   - grandchild " << j+1 << ": " << grandchild->second.segment.getName() << " has joint " << grandchild->second.segment.getJoint().getName() 
           << " and " << grandchild->second.children.size() << " children" << endl;

    }
  }

  // extract chains from tree
  Chain chain1, chain2;
  my_tree.getChain("l_gripper_palm_link", "r_gripper_palm_link", chain1);
  my_tree.getChain("r_gripper_palm_link", "l_gripper_palm_link", chain2);
  cout << "Got chain1 with " << chain1.getNrOfJoints() << " joints and " << chain1.getNrOfSegments() << " segments" << endl;
  cout << "Got chain2 with " << chain2.getNrOfJoints() << " joints and " << chain2.getNrOfSegments() << " segments" << endl;

  JntArray jnt1(chain1.getNrOfJoints());
  JntArray jnt2(chain1.getNrOfJoints());
  for (int i=0; i<(int)chain1.getNrOfJoints(); i++){
    jnt1(i) = (i+1)*2;
    jnt2((int)chain1.getNrOfJoints()-i-1) = -jnt1(i);
  }
  for (int i=0; i<(int)chain1.getNrOfJoints(); i++)
    cout << "jnt 1 -- jnt 2  " << jnt1(i) << " -- " << jnt2(i) << endl;

  ChainFkSolverPos_recursive solver1(chain1);
  ChainFkSolverPos_recursive solver2(chain2);
  Frame f1, f2;
  solver1.JntToCart(jnt1, f1);
  solver2.JntToCart(jnt2, f2);
  cout << "frame 1 " << f1 << endl;
  cout << "frame 2 " << f2.Inverse() << endl;

  // copy tree
  Tree copy = my_tree;
  copy.getChain("l_gripper_palm_link", "r_gripper_palm_link", chain1);
  copy.getChain("r_gripper_palm_link", "l_gripper_palm_link", chain2);
  cout << "Got chain1 with " << chain1.getNrOfJoints() << " joints and " << chain1.getNrOfSegments() << " segments" << endl;
  cout << "Got chain2 with " << chain2.getNrOfJoints() << " joints and " << chain2.getNrOfSegments() << " segments" << endl;

}

