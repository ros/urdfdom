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

#include "test_assets.hpp"
#include "urdf_model/joint.hpp"
#include "urdf_parser/urdf_parser.hpp"

TEST(URDF_SCHEMA_JOINT_DYNAMICS, dynamics_only_damping)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <dynamics damping="5.5"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j1")->dynamics);
  EXPECT_DOUBLE_EQ(5.5, model->getJoint("j1")->dynamics->damping);
  EXPECT_DOUBLE_EQ(0.0, model->getJoint("j1")->dynamics->friction);
}

TEST(URDF_SCHEMA_JOINT_DYNAMICS, dynamics_only_friction)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <dynamics friction="2.25"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j1")->dynamics);
  EXPECT_DOUBLE_EQ(0.0,  model->getJoint("j1")->dynamics->damping);
  EXPECT_DOUBLE_EQ(2.25, model->getJoint("j1")->dynamics->friction);
}

TEST(URDF_SCHEMA_JOINT_DYNAMICS, dynamics_neither_damping_nor_friction_fails)
{
  EXPECT_EQ(nullptr, parseAsset("bad_dynamics_empty.urdf"));
}

TEST(URDF_SCHEMA_JOINT_DYNAMICS, dynamics_negative_damping_allowed_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <dynamics damping="-1.0" friction="0.0"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j1")->dynamics);
  EXPECT_DOUBLE_EQ(-1.0, model->getJoint("j1")->dynamics->damping);
}

TEST(URDF_SCHEMA_JOINT_DYNAMICS, dynamics_negative_friction_allowed_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <dynamics damping="0.0" friction="-0.5"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j1")->dynamics);
  EXPECT_DOUBLE_EQ(-0.5, model->getJoint("j1")->dynamics->friction);
}

TEST(URDF_SCHEMA_JOINT_DYNAMICS, dynamics_zero_values_allowed_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <dynamics damping="0.0" friction="0.0"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j1")->dynamics);
  EXPECT_DOUBLE_EQ(0.0, model->getJoint("j1")->dynamics->damping);
  EXPECT_DOUBLE_EQ(0.0, model->getJoint("j1")->dynamics->friction);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  setlocale(LC_ALL, "");
  return RUN_ALL_TESTS();
}
