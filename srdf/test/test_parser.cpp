/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include <srdf/model.h>
#include <urdf/model.h>
#include <gtest/gtest.h>

TEST(SRDF, Simple)
{
    urdf::Model u;
    srdf::Model s;
    EXPECT_TRUE(u.initFile("test/res/pr2_desc.urdf"));

    EXPECT_TRUE(s.initFile(u, "test/res/pr2_desc.1.srdf"));
    EXPECT_TRUE(s.getVirtualJoints().size() == 0);
    EXPECT_TRUE(s.getGroups().size() == 0);
    EXPECT_TRUE(s.getGroupStates().size() == 0);
    EXPECT_TRUE(s.getDisabledCollisions().empty());
    EXPECT_TRUE(s.getEndEffectors().size() == 0);

    EXPECT_TRUE(s.initFile(u, "test/res/pr2_desc.2.srdf"));
    EXPECT_TRUE(s.getVirtualJoints().size() == 1);
    EXPECT_TRUE(s.getGroups().size() == 1);
    EXPECT_TRUE(s.getGroupStates().size() == 0);
    EXPECT_TRUE(s.getDisabledCollisions().empty());
    EXPECT_TRUE(s.getEndEffectors().size() == 0);

    EXPECT_TRUE(s.initFile(u, "test/res/pr2_desc.1.srdf"));
    EXPECT_TRUE(s.getVirtualJoints().size() == 0);
    EXPECT_TRUE(s.getGroups().size() == 0);
    EXPECT_TRUE(s.getGroupStates().size() == 0);
    EXPECT_TRUE(s.getDisabledCollisions().empty());
    EXPECT_TRUE(s.getEndEffectors().size() == 0);
}

TEST(SRDF, Complex)
{
    urdf::Model u;
    srdf::Model s;
    EXPECT_TRUE(u.initFile("test/res/pr2_desc.urdf"));

    EXPECT_TRUE(s.initFile(u, "test/res/pr2_desc.3.srdf"));
    EXPECT_TRUE(s.getVirtualJoints().size() == 1);
    EXPECT_TRUE(s.getGroups().size() == 7);
    EXPECT_TRUE(s.getGroupStates().size() == 2);
    EXPECT_TRUE(s.getDisabledCollisions().size() == 2);
    EXPECT_TRUE(s.getEndEffectors().size() == 2);

    EXPECT_EQ(s.getVirtualJoints()[0].name_, "world_joint");
    EXPECT_EQ(s.getVirtualJoints()[0].type_, "planar");
    for (std::size_t i = 0 ; i < s.getGroups().size() ; ++i)
    {
        if (s.getGroups()[i].name_ == "left_arm" || s.getGroups()[i].name_ == "right_arm")
            EXPECT_TRUE(s.getGroups()[i].chains_.size() == 1);
        if (s.getGroups()[i].name_ == "arms")
            EXPECT_TRUE(s.getGroups()[i].subgroups_.size() == 2);
        if (s.getGroups()[i].name_ == "base")
            EXPECT_TRUE(s.getGroups()[i].joints_.size() == 1);
        if (s.getGroups()[i].name_ == "l_end_effector" || s.getGroups()[i].name_ == "r_end_effector")
        {
            EXPECT_TRUE(s.getGroups()[i].links_.size() == 1);
            EXPECT_TRUE(s.getGroups()[i].joints_.size() == 9);
        }
        if (s.getGroups()[i].name_ == "whole_body")
        {
            EXPECT_TRUE(s.getGroups()[i].joints_.size() == 1);
            EXPECT_TRUE(s.getGroups()[i].subgroups_.size() == 2);
        }
    }
    int index = 0;
    if (s.getGroupStates()[0].group_ != "arms")
        index = 1;

    EXPECT_EQ(s.getGroupStates()[index].group_, "arms");
    EXPECT_EQ(s.getGroupStates()[index].name_, "tuck_arms");
    EXPECT_EQ(s.getGroupStates()[1-index].group_, "base");
    EXPECT_EQ(s.getGroupStates()[1-index].name_, "home");

    const std::vector<double> &v = s.getGroupStates()[index].joint_values_.find("l_shoulder_pan_joint")->second;
    EXPECT_EQ(v.size(), 1);
    EXPECT_EQ(v[0], 0.2);
    const std::vector<double> &w = s.getGroupStates()[1-index].joint_values_.find("world_joint")->second;
    EXPECT_EQ(w.size(), 3);
    EXPECT_EQ(w[0], 0.4);
    EXPECT_EQ(w[1], 0);
    EXPECT_EQ(w[2], -1);


    index = (s.getEndEffectors()[0].name_[0] == 'r') ? 0 : 1;
    EXPECT_EQ(s.getEndEffectors()[index].name_, "r_end_effector");
    EXPECT_EQ(s.getEndEffectors()[index].component_group_, "r_end_effector");
    EXPECT_EQ(s.getEndEffectors()[index].parent_link_, "r_wrist_roll_link");
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
