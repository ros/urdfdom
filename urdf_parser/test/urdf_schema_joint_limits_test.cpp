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
#include <cmath>
#include <string>

#include "urdf_model/joint.h"
#include "urdf_parser/urdf_parser.h"

TEST(URDF_SCHEMA_JOINT_LIMITS, limits_negative_effort_allowed_v1_0)
{
  // v1.0 does not reject negative effort.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <limit effort="-5.0" velocity="1.0"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_DOUBLE_EQ(-5.0, model->getJoint("j1")->limits->effort);
}

TEST(URDF_SCHEMA_JOINT_LIMITS, limits_negative_velocity_allowed_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <limit effort="10.0" velocity="-3.0"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_DOUBLE_EQ(-3.0, model->getJoint("j1")->limits->velocity);
}

TEST(URDF_SCHEMA_JOINT_LIMITS, limits_zero_effort_and_velocity_allowed_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <limit effort="0.0" velocity="0.0"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_DOUBLE_EQ(0.0, model->getJoint("j1")->limits->effort);
  EXPECT_DOUBLE_EQ(0.0, model->getJoint("j1")->limits->velocity);
}

TEST(URDF_SCHEMA_JOINT_LIMITS, limits_missing_effort_fails_v1_0)
{
  EXPECT_EQ(nullptr, urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_limit_no_effort.urdf"));
}

TEST(URDF_SCHEMA_JOINT_LIMITS, limits_missing_velocity_fails_v1_0)
{
  EXPECT_EQ(nullptr, urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_limit_no_velocity.urdf"));
}

TEST(URDF_SCHEMA_JOINT_LIMITS, limits_large_positive_values_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="revolute">
        <parent link="l1"/>
        <child link="l2"/>
        <limit lower="-3.14159" upper="3.14159"
               effort="9999.9" velocity="100.0"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_NEAR(-3.14159, model->getJoint("j1")->limits->lower,    1e-5);
  EXPECT_NEAR( 3.14159, model->getJoint("j1")->limits->upper,    1e-5);
  EXPECT_NEAR( 9999.9,  model->getJoint("j1")->limits->effort,   1e-5);
  EXPECT_NEAR( 100.0,   model->getJoint("j1")->limits->velocity, 1e-9);
}

TEST(URDF_SCHEMA_JOINT_LIMITS, limits_acceleration_deceleration_jerk_default_to_inf_v1_0)
{
  // These three fields are not a v1.0 concept — they always default to +inf.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <limit effort="10.0" velocity="1.0"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(std::isinf(model->getJoint("j1")->limits->acceleration));
  EXPECT_TRUE(std::isinf(model->getJoint("j1")->limits->deceleration));
  EXPECT_TRUE(std::isinf(model->getJoint("j1")->limits->jerk));
}

TEST(URDF_SCHEMA_JOINT_LIMITS, limits_invalid_effort_string_fails_v1_0)
{
  EXPECT_EQ(nullptr, urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_limit_effort_not_number.urdf"));
}

TEST(URDF_SCHEMA_JOINT_LIMITS, limits_invalid_velocity_string_fails_v1_0)
{
  EXPECT_EQ(nullptr, urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_limit_velocity_not_number.urdf"));
}

TEST(URDF_SCHEMA_JOINT_LIMITS, revolute_joint_limits_lower_greater_than_upper_allowed_v1_0)
{
  // In v1.0 the lower > upper check is not performed — parsed without error.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="revolute">
        <parent link="l1"/>
        <child link="l2"/>
        <limit lower="1.57" upper="-1.57" effort="10.0" velocity="1.0"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j1")->limits);
  EXPECT_DOUBLE_EQ( 1.57, model->getJoint("j1")->limits->lower);
  EXPECT_DOUBLE_EQ(-1.57, model->getJoint("j1")->limits->upper);
}

TEST(URDF_SCHEMA_JOINT_LIMITS, prismatic_joint_limits_lower_greater_than_upper_allowed_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="prismatic">
        <parent link="l1"/>
        <child link="l2"/>
        <limit lower="0.5" upper="-0.5" effort="50.0" velocity="0.5"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j1")->limits);
  EXPECT_DOUBLE_EQ( 0.5, model->getJoint("j1")->limits->lower);
  EXPECT_DOUBLE_EQ(-0.5, model->getJoint("j1")->limits->upper);
}

TEST(URDF_SCHEMA_JOINT_LIMITS, revolute_joint_negative_effort_allowed_v1_0)
{
  // v1.0 does not validate the sign of effort on revolute joints.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="revolute">
        <parent link="l1"/>
        <child link="l2"/>
        <limit lower="-1.57" upper="1.57" effort="-10.0" velocity="1.0"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j1")->limits);
  EXPECT_DOUBLE_EQ(-10.0, model->getJoint("j1")->limits->effort);
}

TEST(URDF_SCHEMA_JOINT_LIMITS, revolute_joint_negative_velocity_allowed_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="revolute">
        <parent link="l1"/>
        <child link="l2"/>
        <limit lower="-1.57" upper="1.57" effort="10.0" velocity="-2.0"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j1")->limits);
  EXPECT_DOUBLE_EQ(-2.0, model->getJoint("j1")->limits->velocity);
}

TEST(URDF_SCHEMA_JOINT_LIMITS, prismatic_joint_negative_effort_and_velocity_allowed_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="prismatic">
        <parent link="l1"/>
        <child link="l2"/>
        <limit lower="-0.5" upper="0.5" effort="-50.0" velocity="-0.5"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j1")->limits);
  EXPECT_DOUBLE_EQ(-50.0, model->getJoint("j1")->limits->effort);
  EXPECT_DOUBLE_EQ( -0.5, model->getJoint("j1")->limits->velocity);
}

TEST(URDF_SCHEMA_JOINT_LIMITS, revolute_joint_zero_effort_and_velocity_allowed_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="revolute">
        <parent link="l1"/>
        <child link="l2"/>
        <limit lower="-1.0" upper="1.0" effort="0.0" velocity="0.0"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_NE(nullptr, model->getJoint("j1")->limits);
  EXPECT_DOUBLE_EQ(0.0, model->getJoint("j1")->limits->effort);
  EXPECT_DOUBLE_EQ(0.0, model->getJoint("j1")->limits->velocity);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  setlocale(LC_ALL, "");
  return RUN_ALL_TESTS();
}
