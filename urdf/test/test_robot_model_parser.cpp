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

protected:
  /// constructor
  TestParser()
  {
  }


  /// Destructor
  ~TestParser()
  {
  }
};




TEST_F(TestParser, test)
{
  for (int i=1; i<g_argc; i++){
    printf("\n\nprocessing file %d : %s\n",i,g_argv[i]);
    TiXmlDocument robot_model_xml;
    robot_model_xml.LoadFile(g_argv[i]);
    TiXmlElement *robot_xml = robot_model_xml.FirstChildElement("robot");
    ASSERT_TRUE(robot_xml != NULL);
    if (i == g_argc-1)
      ASSERT_TRUE(robot.initXml(robot_xml));
    else
      ASSERT_FALSE(robot.initXml(robot_xml));
  }
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
