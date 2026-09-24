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

#include "urdf_parser/urdf_parser.h"

TEST(URDF_SCHEMA_MATERIAL, global_material_with_color)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/global_material.urdf");
  ASSERT_NE(nullptr, model);
  auto mat = model->getMaterial("blue");
  ASSERT_NE(nullptr, mat);
  EXPECT_FLOAT_EQ(0.0f, static_cast<float>(mat->color.r));
  EXPECT_FLOAT_EQ(0.0f, static_cast<float>(mat->color.g));
  EXPECT_FLOAT_EQ(1.0f, static_cast<float>(mat->color.b));
  EXPECT_FLOAT_EQ(1.0f, static_cast<float>(mat->color.a));
}

TEST(URDF_SCHEMA_MATERIAL, global_material_with_texture)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <material name="checker">
        <texture filename="package://my_pkg/textures/checker.png"/>
      </material>
      <link name="base">
        <visual>
          <geometry><sphere radius="0.1"/></geometry>
          <material name="checker"/>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto mat = model->getMaterial("checker");
  ASSERT_NE(nullptr, mat);
  EXPECT_EQ("package://my_pkg/textures/checker.png", mat->texture_filename);
}

TEST(URDF_SCHEMA_MATERIAL, duplicate_global_material_fails)
{
  EXPECT_EQ(nullptr, urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_duplicate_material.urdf"));
}

TEST(URDF_SCHEMA_MATERIAL, inline_material_definition_in_visual)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <geometry><sphere radius="0.1"/></geometry>
          <material name="green_ish">
            <color rgba="0.1 0.9 0.1 0.5"/>
          </material>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_FALSE(model->getLink("base")->visual_array.empty());
  auto vis = model->getLink("base")->visual_array[0];
  ASSERT_NE(nullptr, vis->material);
  EXPECT_EQ("green_ish", vis->material->name);
  EXPECT_FLOAT_EQ(0.1f, static_cast<float>(vis->material->color.r));
  EXPECT_FLOAT_EQ(0.9f, static_cast<float>(vis->material->color.g));
  EXPECT_FLOAT_EQ(0.5f, static_cast<float>(vis->material->color.a));
}

TEST(URDF_SCHEMA_MATERIAL, visual_color_rgba_boundary_values)
{
  // Test rgba with 0.0 and 1.0 boundary values
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <geometry><sphere radius="0.1"/></geometry>
          <material name="">
            <color rgba="0.0 1.0 0.0 1.0"/>
          </material>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto vis = model->getLink("base")->visual_array[0];
  ASSERT_NE(nullptr, vis->material);
  EXPECT_FLOAT_EQ(0.0f, static_cast<float>(vis->material->color.r));
  EXPECT_FLOAT_EQ(1.0f, static_cast<float>(vis->material->color.g));
  EXPECT_FLOAT_EQ(0.0f, static_cast<float>(vis->material->color.b));
  EXPECT_FLOAT_EQ(1.0f, static_cast<float>(vis->material->color.a));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  setlocale(LC_ALL, "");
  return RUN_ALL_TESTS();
}
