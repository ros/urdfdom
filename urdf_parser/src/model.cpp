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

/* Author: John Hsu */


#include <urdf_model/model.h>
#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <urdf_parser/exceptions.h>
#include <urdf_parser/console.h>

namespace urdf{

bool ModelInterface::initTree(std::map<std::string, std::string> &parent_link_tree)
{
  // loop through all joints, for every link, assign children links and children joints
  for (std::map<std::string,boost::shared_ptr<Joint> >::iterator joint = this->joints_.begin();joint != this->joints_.end(); joint++)
  {
    std::string parent_link_name = joint->second->parent_link_name;
    std::string child_link_name = joint->second->child_link_name;

    logDebug("build tree: joint: '%s' has parent link '%s' and child  link '%s'", joint->first.c_str(), parent_link_name.c_str(),child_link_name.c_str());
    if (parent_link_name.empty() || child_link_name.empty())
    {
      logError("    Joint %s is missing a parent and/or child link specification.",(joint->second)->name.c_str());
      return false;
    }
    else
    {
      // find child and parent links
      boost::shared_ptr<Link> child_link, parent_link;
      this->getLink(child_link_name, child_link);
      if (!child_link)
      {
        logError("    child link '%s' of joint '%s' not found", child_link_name.c_str(), joint->first.c_str() );
        return false;
      }
      this->getLink(parent_link_name, parent_link);
      if (!parent_link)
      {
        logError("    parent link '%s' of joint '%s' not found.  This is not valid according to the URDF spec. Every link you refer to from a joint needs to be explicitly defined in the robot description. To fix this problem you can either remove this joint [%s] from your urdf file, or add \"<link name=\"%s\" />\" to your urdf file.", parent_link_name.c_str(), joint->first.c_str(), joint->first.c_str(), parent_link_name.c_str() );
        return false;
      }

      //set parent link for child link
      child_link->setParent(parent_link);

      //set parent joint for child link
      child_link->setParentJoint(joint->second);

      //set child joint for parent link
      parent_link->addChildJoint(joint->second);
    
      //set child link for parent link
      parent_link->addChild(child_link);
    
      // fill in child/parent string map
      parent_link_tree[child_link->name] = parent_link_name;
    
      logDebug("    now Link '%s' has %i children ", parent_link->name.c_str(), (int)parent_link->child_links.size());
    }
  }

  return true;
};

bool ModelInterface::initRoot(std::map<std::string, std::string> &parent_link_tree)
{
  this->root_link_.reset();

  // find the links that have no parent in the tree
  for (std::map<std::string, boost::shared_ptr<Link> >::iterator l=this->links_.begin(); l!=this->links_.end(); l++)  
  {
    std::map<std::string, std::string >::iterator parent = parent_link_tree.find(l->first);
    if (parent == parent_link_tree.end())
    {
      // store root link
      if (!this->root_link_)
      {
        getLink(l->first, this->root_link_);
      }
      // we already found a root link
      else{
        logError("Two root links found: '%s' and '%s'", this->root_link_->name.c_str(), l->first.c_str());
        return false;
      }
    }
  }
  if (!this->root_link_)
  {
    logError("No root link found. The robot xml is not a valid tree.");
    return false;
  }
  logDebug("Link '%s' is the root link", this->root_link_->name.c_str());

  return true;
};

}


