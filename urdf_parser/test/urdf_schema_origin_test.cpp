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
#include "urdf_model/pose.h"
#include "urdf_parser/urdf_parser.h"

TEST(URDF_SCHEMA_ORIGIN, origin_no_attributes_is_identity)
{
  // Missing xyz/rpy both default to zero → identity.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <origin/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  const auto & tf = model->getJoint("j1")->parent_to_joint_origin_transform;
  EXPECT_DOUBLE_EQ(0.0, tf.position.x);
  EXPECT_DOUBLE_EQ(0.0, tf.position.y);
  EXPECT_DOUBLE_EQ(0.0, tf.position.z);
  double x, y, z, w;
  tf.rotation.getQuaternion(x, y, z, w);
  EXPECT_DOUBLE_EQ(0.0, x);
  EXPECT_DOUBLE_EQ(0.0, y);
  EXPECT_DOUBLE_EQ(0.0, z);
  EXPECT_DOUBLE_EQ(1.0, w);
}

TEST(URDF_SCHEMA_ORIGIN, origin_xyz_only)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <origin xyz="0.5 -1.5 2.25"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  const auto & pos = model->getJoint("j1")->parent_to_joint_origin_transform.position;
  EXPECT_NEAR( 0.5,  pos.x, 1e-9);
  EXPECT_NEAR(-1.5,  pos.y, 1e-9);
  EXPECT_NEAR( 2.25, pos.z, 1e-9);
}

TEST(URDF_SCHEMA_ORIGIN, origin_rpy_only)
{
  // rpy="pi/2 0 0" → rotation about X by 90°
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <origin rpy="1.5707963 0 0"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  const auto & rot = model->getJoint("j1")->parent_to_joint_origin_transform.rotation;
  double x, y, z, w;
  rot.getQuaternion(x, y, z, w);
  EXPECT_NEAR(0.7071068, x, 1e-5);
  EXPECT_NEAR(0.0,       y, 1e-5);
  EXPECT_NEAR(0.0,       z, 1e-5);
  EXPECT_NEAR(0.7071068, w, 1e-5);
}

TEST(URDF_SCHEMA_ORIGIN, origin_negative_xyz_values)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <origin xyz="-10.0 -20.5 -0.001"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  const auto & pos = model->getJoint("j1")->parent_to_joint_origin_transform.position;
  EXPECT_NEAR(-10.0,   pos.x, 1e-9);
  EXPECT_NEAR(-20.5,   pos.y, 1e-9);
  EXPECT_NEAR(-0.001,  pos.z, 1e-9);
}

TEST(URDF_SCHEMA_ORIGIN, quat_xyzw_ignored_in_v1_0)
{
  // quat_xyzw on an origin element is silently ignored in v1.0.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <origin quat_xyzw="0.5 0.5 0.5 0.5"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  double x, y, z, w;
  model->getJoint("j1")->parent_to_joint_origin_transform.rotation.getQuaternion(x, y, z, w);
  EXPECT_DOUBLE_EQ(0.0, x);
  EXPECT_DOUBLE_EQ(0.0, y);
  EXPECT_DOUBLE_EQ(0.0, z);
  EXPECT_DOUBLE_EQ(1.0, w);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  setlocale(LC_ALL, "");
  return RUN_ALL_TESTS();
}
