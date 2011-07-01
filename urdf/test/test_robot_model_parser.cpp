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

#include <string>
#include <gtest/gtest.h>
#include "urdf/model.h"

// Including ros, just to be able to call ros::init(), to remove unwanted
// args from command-line.
#include <ros/ros.h>

using namespace urdf;

int g_argc;
char** g_argv;

class TestParser : public testing::Test
{
public:
  Model robot;

  bool checkModel()
  {
    // get root link
    boost::shared_ptr<const Link> root_link=this->robot.getRoot();
    if (!root_link)
    {
      ROS_ERROR("no root link %s",this->robot.getName().c_str());
      return false;
    }

    // go through entire tree
    return this->traverse_tree(root_link); 

  };

protected:
  /// constructor
  TestParser()
  {
  }


  /// Destructor
  ~TestParser()
  {
  }

  bool traverse_tree(boost::shared_ptr<const Link> link,int level = 0)
  {
    level+=2;
    for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
    {
      if (*child)
      {
        // check rpy
        double roll,pitch,yaw;
        (*child)->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(roll,pitch,yaw);

        if (isnan(roll) || isnan(pitch) || isnan(yaw))
        {
          ROS_ERROR("getRPY() returned nan!");
          return false;
        }
        // recurse down the tree
        return this->traverse_tree(*child,level);
      }
      else
      {
        ROS_ERROR("root link: %s has a null child!",link->name.c_str());
        return false;
      }
    }
    // no children
    return true;
  };
};




TEST_F(TestParser, test)
{
  std::string folder = std::string(g_argv[1]) + "/test/";
  ROS_INFO("Folder %s",folder.c_str());
  for (int i=2; i<g_argc; i++){
    std::string file = g_argv[i];
    bool expect_success = (file.substr(0,5)  != "fail_");
    ROS_INFO("Parsing file %d/%d called %s, expecting %d",(i-1), g_argc-1, (folder + file).c_str(), expect_success);
    if (!expect_success)
      ASSERT_FALSE(robot.initFile(folder + file));
    else
    {
      ASSERT_TRUE(robot.initFile(folder + file));
      ASSERT_TRUE(checkModel());
    }
  }

  // test reading from parameter server
  ASSERT_TRUE(robot.initParam("robot_description"));
  ASSERT_FALSE(robot.initParam("robot_description_wim"));
  SUCCEED();
}




int main(int argc, char** argv)
{
  // Calling ros::init(), just to remove unwanted args from command-line.
  ros::init(argc, argv, "test", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
