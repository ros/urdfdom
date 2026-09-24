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

#include "urdf_model/link.h"
#include "urdf_parser/urdf_parser.h"

TEST(URDF_SCHEMA_LINK_INERTIAL, inertial_positive_values)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/inertial_link.urdf");
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_NE(nullptr, link);
  ASSERT_NE(nullptr, link->inertial);
  EXPECT_DOUBLE_EQ(5.25,  link->inertial->mass);
  EXPECT_DOUBLE_EQ(0.2,   link->inertial->ixx);
  EXPECT_DOUBLE_EQ(0.01,  link->inertial->ixy);
  EXPECT_DOUBLE_EQ(0.001, link->inertial->ixz);
  EXPECT_DOUBLE_EQ(0.3,   link->inertial->iyy);
  EXPECT_DOUBLE_EQ(0.002, link->inertial->iyz);
  EXPECT_DOUBLE_EQ(0.4,   link->inertial->izz);
}

TEST(URDF_SCHEMA_LINK_INERTIAL, inertial_negative_mass_allowed_v1_0)
{
  // v1.0 applies no sign validation on mass.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <inertial>
          <mass value="-3.0"/>
          <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_NE(nullptr, link);
  ASSERT_NE(nullptr, link->inertial);
  EXPECT_DOUBLE_EQ(-3.0, link->inertial->mass);
}

TEST(URDF_SCHEMA_LINK_INERTIAL, inertial_zero_mass_allowed_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <inertial>
          <mass value="0.0"/>
          <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
        </inertial>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_NE(nullptr, link);
  ASSERT_NE(nullptr, link->inertial);
  EXPECT_DOUBLE_EQ(0.0, link->inertial->mass);
}

TEST(URDF_SCHEMA_LINK_INERTIAL, inertial_negative_inertia_tensor_allowed_v1_0)
{
  // Negative off-diagonal terms are physically allowed; v1.0 doesn't validate.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <inertial>
          <mass value="1.0"/>
          <inertia ixx="0.5" ixy="-0.1" ixz="-0.2"
                   iyy="0.6" iyz="-0.3" izz="0.7"/>
        </inertial>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_NE(nullptr, link);
  ASSERT_NE(nullptr, link->inertial);
  EXPECT_DOUBLE_EQ(-0.1, link->inertial->ixy);
  EXPECT_DOUBLE_EQ(-0.2, link->inertial->ixz);
  EXPECT_DOUBLE_EQ(-0.3, link->inertial->iyz);
}

TEST(URDF_SCHEMA_LINK_INERTIAL, inertial_with_origin)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <inertial>
          <origin xyz="0.1 -0.2 0.3" rpy="0 0 1.5707963"/>
          <mass value="2.0"/>
          <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_NE(nullptr, link->inertial);
  EXPECT_NEAR(0.1,  link->inertial->origin.position.x, 1e-9);
  EXPECT_NEAR(-0.2, link->inertial->origin.position.y, 1e-9);
  EXPECT_NEAR(0.3,  link->inertial->origin.position.z, 1e-9);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  setlocale(LC_ALL, "");
  return RUN_ALL_TESTS();
}
