// Copyright 2026 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <string>

#include "urdf_model/joint.h"
#include "urdf_parser/urdf_parser.h"

TEST(URDF_SCHEMA_JOINT_MIMIC, mimic_joint_name_only_uses_defaults)
{
  // When multiplier and offset are absent they default to 1 and 0 respectively.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <link name="l3"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
      </joint>
      <joint name="j2" type="fixed">
        <parent link="l1"/>
        <child link="l3"/>
        <mimic joint="j1"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j2")->mimic);
  EXPECT_EQ("j1",  model->getJoint("j2")->mimic->joint_name);
  EXPECT_DOUBLE_EQ(1.0, model->getJoint("j2")->mimic->multiplier);
  EXPECT_DOUBLE_EQ(0.0, model->getJoint("j2")->mimic->offset);
}

TEST(URDF_SCHEMA_JOINT_MIMIC, mimic_with_negative_multiplier_allowed_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <link name="l3"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
      </joint>
      <joint name="j2" type="fixed">
        <parent link="l1"/>
        <child link="l3"/>
        <mimic joint="j1" multiplier="-2.0" offset="0.1"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j2")->mimic);
  EXPECT_DOUBLE_EQ(-2.0, model->getJoint("j2")->mimic->multiplier);
  EXPECT_DOUBLE_EQ( 0.1, model->getJoint("j2")->mimic->offset);
}

TEST(URDF_SCHEMA_JOINT_MIMIC, mimic_without_joint_name_fails)
{
  EXPECT_EQ(nullptr, urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_mimic_no_joint_name.urdf"));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  setlocale(LC_ALL, "");
  return RUN_ALL_TESTS();
}
