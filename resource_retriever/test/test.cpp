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

#include <gtest/gtest.h>

#include <resource_retriever/retriever.h>
#include <ros/package.h>
#include <ros/console.h>

using namespace resource_retriever;

TEST(Retriever, getByPackage)
{
  try
  {
    Retriever r;
    MemoryResource res = r.get("package://"ROS_PACKAGE_NAME"/test/test.txt");

    ASSERT_EQ(res.size, 1);
    ASSERT_EQ(res.data[0], 'A');
  }
  catch (Exception& e)
  {
    FAIL();
  }
}

TEST(Retriever, largeFile)
{
  try
  {
    std::string path = ros::package::getPath(ROS_PACKAGE_NAME) + "/test/large_file.dat";

    FILE* f = fopen(path.c_str(), "w");

    ASSERT_TRUE(f);

    for (int i = 0; i < 1024*1024*50; ++i)
    {
      fprintf(f, "A");
    }
    fclose(f);

    Retriever r;
    MemoryResource res = r.get("package://"ROS_PACKAGE_NAME"/test/large_file.dat");

    ASSERT_EQ(res.size, 1024*1024*50);
  }
  catch (Exception& e)
  {
    FAIL();
  }
}

TEST(Retriever, http)
{
  try
  {
    Retriever r;
    MemoryResource res = r.get("http://pr.willowgarage.com/downloads/svnmerge.py");

    ASSERT_GT(res.size, 0);
  }
  catch (Exception& e)
  {
    FAIL();
  }
}

TEST(Retriever, invalidFiles)
{
  Retriever r;

  try
  {
    r.get("file://fail");
    FAIL();
  }
  catch (Exception& e)
  {
    ROS_INFO("%s", e.what());
  }

  try
  {
    r.get("package://roscpp");
    FAIL();
  }
  catch (Exception& e)
  {
    ROS_INFO("%s", e.what());
  }

  try
  {
    r.get("package://invalid_package_blah/test.xml");
    FAIL();
  }
  catch (Exception& e)
  {
    ROS_INFO("%s", e.what());
  }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

