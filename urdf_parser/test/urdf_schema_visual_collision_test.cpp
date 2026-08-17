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

TEST(URDF_SCHEMA_VISUAL_COLLISION, visual_with_name_attr)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual name="my_visual">
          <geometry><sphere radius="0.1"/></geometry>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ(1u, model->getLink("base")->visual_array.size());
  EXPECT_EQ("my_visual", model->getLink("base")->visual_array[0]->name);
}

TEST(URDF_SCHEMA_VISUAL_COLLISION, collision_with_name_attr)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <collision name="my_col">
          <geometry><sphere radius="0.1"/></geometry>
        </collision>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ(1u, model->getLink("base")->collision_array.size());
  EXPECT_EQ("my_col", model->getLink("base")->collision_array[0]->name);
}

TEST(URDF_SCHEMA_VISUAL_COLLISION, visual_with_origin)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <origin xyz="1.0 -2.0 3.0" rpy="0.1 0.2 0.3"/>
          <geometry><sphere radius="0.1"/></geometry>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_FALSE(model->getLink("base")->visual_array.empty());
  const auto & origin = model->getLink("base")->visual_array[0]->origin;
  EXPECT_NEAR( 1.0, origin.position.x, 1e-9);
  EXPECT_NEAR(-2.0, origin.position.y, 1e-9);
  EXPECT_NEAR( 3.0, origin.position.z, 1e-9);
}

TEST(URDF_SCHEMA_VISUAL_COLLISION, collision_with_origin)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <collision>
          <origin xyz="-0.5 0.0 0.25" rpy="0 0 0"/>
          <geometry><box size="1 1 1"/></geometry>
        </collision>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_FALSE(model->getLink("base")->collision_array.empty());
  const auto & origin = model->getLink("base")->collision_array[0]->origin;
  EXPECT_NEAR(-0.5,  origin.position.x, 1e-9);
  EXPECT_NEAR( 0.0,  origin.position.y, 1e-9);
  EXPECT_NEAR( 0.25, origin.position.z, 1e-9);
}

TEST(URDF_SCHEMA_VISUAL_COLLISION, multiple_visuals_v1_0)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/multiple_geometries.urdf");
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_EQ(3u, link->visual_array.size());
  EXPECT_EQ(urdf::Geometry::SPHERE,   link->visual_array[0]->geometry->type);
  EXPECT_EQ(urdf::Geometry::BOX,      link->visual_array[1]->geometry->type);
  EXPECT_EQ(urdf::Geometry::CYLINDER, link->visual_array[2]->geometry->type);
  // .visual pointer must point to the first element
  EXPECT_EQ(link->visual_array[0], link->visual);
}

TEST(URDF_SCHEMA_VISUAL_COLLISION, multiple_collisions_v1_0)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/multiple_geometries.urdf");
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_EQ(2u, link->collision_array.size());
  EXPECT_EQ(link->collision_array[0], link->collision);
}

TEST(URDF_SCHEMA_VISUAL_COLLISION, visual_and_collision_in_same_link)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <geometry><sphere radius="0.5"/></geometry>
        </visual>
        <collision>
          <geometry><box size="1 1 1"/></geometry>
        </collision>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  EXPECT_EQ(1u, link->visual_array.size());
  EXPECT_EQ(1u, link->collision_array.size());
  EXPECT_EQ(urdf::Geometry::SPHERE, link->visual->geometry->type);
  EXPECT_EQ(urdf::Geometry::BOX,    link->collision->geometry->type);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  setlocale(LC_ALL, "");
  return RUN_ALL_TESTS();
}
