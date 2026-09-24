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

TEST(URDF_SCHEMA_JOINT_TYPES, revolute_joint_with_limits)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/revolute_joint.urdf");
  ASSERT_NE(nullptr, model);
  EXPECT_EQ(urdf::Joint::REVOLUTE, model->getJoint("j1")->type);
  ASSERT_NE(nullptr, model->getJoint("j1")->limits);
  EXPECT_DOUBLE_EQ(-1.57,  model->getJoint("j1")->limits->lower);
  EXPECT_DOUBLE_EQ( 1.57,  model->getJoint("j1")->limits->upper);
  EXPECT_DOUBLE_EQ(100.0,  model->getJoint("j1")->limits->effort);
  EXPECT_DOUBLE_EQ(  2.0,  model->getJoint("j1")->limits->velocity);
}

TEST(URDF_SCHEMA_JOINT_TYPES, revolute_joint_without_limits_fails)
{
  EXPECT_EQ(nullptr, urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_revolute_no_limits.urdf"));
}

TEST(URDF_SCHEMA_JOINT_TYPES, prismatic_joint_with_limits)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="prismatic">
        <parent link="l1"/>
        <child link="l2"/>
        <axis xyz="0 0 1"/>
        <limit lower="-0.5" upper="0.5" effort="50.0" velocity="0.5"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ(urdf::Joint::PRISMATIC, model->getJoint("j1")->type);
}

TEST(URDF_SCHEMA_JOINT_TYPES, prismatic_joint_without_limits_fails)
{
  EXPECT_EQ(nullptr, urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_prismatic_no_limits.urdf"));
}

TEST(URDF_SCHEMA_JOINT_TYPES, continuous_joint_no_limits_required)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="continuous">
        <parent link="l1"/>
        <child link="l2"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ(urdf::Joint::CONTINUOUS, model->getJoint("j1")->type);
  EXPECT_EQ(nullptr, model->getJoint("j1")->limits);
}

TEST(URDF_SCHEMA_JOINT_TYPES, fixed_joint_no_limits_required)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ(urdf::Joint::FIXED, model->getJoint("j1")->type);
  EXPECT_EQ(nullptr, model->getJoint("j1")->limits);
}

TEST(URDF_SCHEMA_JOINT_TYPES, floating_joint_no_limits_required)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="floating">
        <parent link="l1"/>
        <child link="l2"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ(urdf::Joint::FLOATING, model->getJoint("j1")->type);
}

TEST(URDF_SCHEMA_JOINT_TYPES, planar_joint_no_limits_required)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="planar">
        <parent link="l1"/>
        <child link="l2"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ(urdf::Joint::PLANAR, model->getJoint("j1")->type);
}

TEST(URDF_SCHEMA_JOINT_TYPES, unknown_joint_type_fails)
{
  EXPECT_EQ(nullptr, urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_unknown_joint_type.urdf"));
}

TEST(URDF_SCHEMA_JOINT_TYPES, joint_without_type_attr_fails)
{
  EXPECT_EQ(nullptr, urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_joint_no_type.urdf"));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  setlocale(LC_ALL, "");
  return RUN_ALL_TESTS();
}
