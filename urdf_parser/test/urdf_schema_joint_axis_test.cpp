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

TEST(URDF_SCHEMA_JOINT_AXIS, joint_axis_default_is_1_0_0)
{
  // When <axis> is absent, default is (1,0,0).
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="revolute">
        <parent link="l1"/>
        <child link="l2"/>
        <limit lower="-1" upper="1" effort="10" velocity="1"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  const auto & ax = model->getJoint("j1")->axis;
  EXPECT_DOUBLE_EQ(1.0, ax.x);
  EXPECT_DOUBLE_EQ(0.0, ax.y);
  EXPECT_DOUBLE_EQ(0.0, ax.z);
}

TEST(URDF_SCHEMA_JOINT_AXIS, joint_axis_custom_value)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="revolute">
        <parent link="l1"/>
        <child link="l2"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1" upper="1" effort="10" velocity="1"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  const auto & ax = model->getJoint("j1")->axis;
  EXPECT_DOUBLE_EQ(0.0, ax.x);
  EXPECT_DOUBLE_EQ(1.0, ax.y);
  EXPECT_DOUBLE_EQ(0.0, ax.z);
}

TEST(URDF_SCHEMA_JOINT_AXIS, joint_axis_negative_components)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="continuous">
        <parent link="l1"/>
        <child link="l2"/>
        <axis xyz="0 0 -1"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  const auto & ax = model->getJoint("j1")->axis;
  EXPECT_DOUBLE_EQ( 0.0, ax.x);
  EXPECT_DOUBLE_EQ( 0.0, ax.y);
  EXPECT_DOUBLE_EQ(-1.0, ax.z);
}

TEST(URDF_SCHEMA_JOINT_AXIS, joint_axis_zero_vector_allowed_v1_0)
{
  // v1.0 does not reject a zero-length axis — it is stored verbatim.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="continuous">
        <parent link="l1"/>
        <child link="l2"/>
        <axis xyz="0 0 0"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  const auto & ax = model->getJoint("j1")->axis;
  EXPECT_DOUBLE_EQ(0.0, ax.x);
  EXPECT_DOUBLE_EQ(0.0, ax.y);
  EXPECT_DOUBLE_EQ(0.0, ax.z);
}

TEST(URDF_SCHEMA_JOINT_AXIS, joint_axis_non_unit_length_allowed_v1_0)
{
  // v1.0 does not normalise the axis — non-unit vectors are stored as-is.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="revolute">
        <parent link="l1"/>
        <child link="l2"/>
        <axis xyz="2 0 0"/>
        <limit lower="-1" upper="1" effort="10" velocity="1"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  const auto & ax = model->getJoint("j1")->axis;
  EXPECT_DOUBLE_EQ(2.0, ax.x);
  EXPECT_DOUBLE_EQ(0.0, ax.y);
  EXPECT_DOUBLE_EQ(0.0, ax.z);
}

TEST(URDF_SCHEMA_JOINT_AXIS, joint_axis_non_principal_unit_vector_allowed)
{
  // A unit vector not aligned with X/Y/Z is valid and stored verbatim.
  // 1/sqrt(3) ≈ 0.577350269...
  const double c = 1.0 / std::sqrt(3.0);
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="revolute">
        <parent link="l1"/>
        <child link="l2"/>
        <axis xyz="0.57735026919 0.57735026919 0.57735026919"/>
        <limit lower="-1" upper="1" effort="10" velocity="1"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  const auto & ax = model->getJoint("j1")->axis;
  EXPECT_NEAR(c, ax.x, 1e-8);
  EXPECT_NEAR(c, ax.y, 1e-8);
  EXPECT_NEAR(c, ax.z, 1e-8);
  EXPECT_NEAR(1.0, std::sqrt(ax.x * ax.x + ax.y * ax.y + ax.z * ax.z), 1e-7);
}

TEST(URDF_SCHEMA_JOINT_AXIS, joint_axis_diagonal_unit_vector_xy_plane)
{
  // 45-degree axis in the XY plane: (1/sqrt(2), 1/sqrt(2), 0).
  const double c = 1.0 / std::sqrt(2.0);
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="continuous">
        <parent link="l1"/>
        <child link="l2"/>
        <axis xyz="0.70710678118 0.70710678118 0"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  const auto & ax = model->getJoint("j1")->axis;
  EXPECT_NEAR(c,   ax.x, 1e-8);
  EXPECT_NEAR(c,   ax.y, 1e-8);
  EXPECT_NEAR(0.0, ax.z, 1e-8);
  EXPECT_NEAR(1.0, std::sqrt(ax.x * ax.x + ax.y * ax.y + ax.z * ax.z), 1e-7);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  setlocale(LC_ALL, "");
  return RUN_ALL_TESTS();
}
