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

TEST(URDF_SCHEMA_JOINT_CALIBRATION, calibration_rising_only)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <calibration rising="1.234"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j1")->calibration);
  ASSERT_NE(nullptr, model->getJoint("j1")->calibration->rising);
  EXPECT_DOUBLE_EQ(1.234, *model->getJoint("j1")->calibration->rising);
  EXPECT_EQ(nullptr, model->getJoint("j1")->calibration->falling);
}

TEST(URDF_SCHEMA_JOINT_CALIBRATION, calibration_falling_only)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <calibration falling="-0.567"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j1")->calibration);
  EXPECT_EQ(nullptr, model->getJoint("j1")->calibration->rising);
  ASSERT_NE(nullptr, model->getJoint("j1")->calibration->falling);
  EXPECT_DOUBLE_EQ(-0.567, *model->getJoint("j1")->calibration->falling);
}

TEST(URDF_SCHEMA_JOINT_CALIBRATION, calibration_both_rising_and_falling)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <calibration rising="0.5" falling="-0.5"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j1")->calibration->rising);
  ASSERT_NE(nullptr, model->getJoint("j1")->calibration->falling);
  EXPECT_DOUBLE_EQ( 0.5, *model->getJoint("j1")->calibration->rising);
  EXPECT_DOUBLE_EQ(-0.5, *model->getJoint("j1")->calibration->falling);
}

TEST(URDF_SCHEMA_JOINT_CALIBRATION, calibration_no_attributes_produces_null_pointers)
{
  // <calibration/> with no attributes is valid — both optional.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <calibration/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j1")->calibration);
  EXPECT_EQ(nullptr, model->getJoint("j1")->calibration->rising);
  EXPECT_EQ(nullptr, model->getJoint("j1")->calibration->falling);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  setlocale(LC_ALL, "");
  return RUN_ALL_TESTS();
}
