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

TEST(URDF_SCHEMA_JOINT_SAFETY, safety_k_velocity_only_other_defaults_to_zero)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <safety_controller k_velocity="8.5"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j1")->safety);
  EXPECT_DOUBLE_EQ(8.5, model->getJoint("j1")->safety->k_velocity);
  EXPECT_DOUBLE_EQ(0.0, model->getJoint("j1")->safety->k_position);
  EXPECT_DOUBLE_EQ(0.0, model->getJoint("j1")->safety->soft_lower_limit);
  EXPECT_DOUBLE_EQ(0.0, model->getJoint("j1")->safety->soft_upper_limit);
}

TEST(URDF_SCHEMA_JOINT_SAFETY, safety_without_k_velocity_fails)
{
  EXPECT_EQ(nullptr, urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_safety_no_k_velocity.urdf"));
}

TEST(URDF_SCHEMA_JOINT_SAFETY, safety_negative_soft_limits_allowed_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <safety_controller soft_lower_limit="-5.0" soft_upper_limit="-1.0"
                           k_position="0.1" k_velocity="3.0"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_DOUBLE_EQ(-5.0, model->getJoint("j1")->safety->soft_lower_limit);
  EXPECT_DOUBLE_EQ(-1.0, model->getJoint("j1")->safety->soft_upper_limit);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  setlocale(LC_ALL, "");
  return RUN_ALL_TESTS();
}
