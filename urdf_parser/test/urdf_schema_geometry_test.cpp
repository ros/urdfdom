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
#include <memory>
#include <string>

#include "urdf_model/link.h"
#include "urdf_parser/urdf_parser.h"

TEST(URDF_SCHEMA_GEOMETRY, box_geometry_positive_values)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <geometry><box size="1.0 2.0 3.0"/></geometry>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_FALSE(link->visual_array.empty());
  EXPECT_EQ(urdf::Geometry::BOX, link->visual_array[0]->geometry->type);
  auto box = std::dynamic_pointer_cast<urdf::Box>(link->visual_array[0]->geometry);
  ASSERT_NE(nullptr, box);
  EXPECT_DOUBLE_EQ(1.0, box->dim.x);
  EXPECT_DOUBLE_EQ(2.0, box->dim.y);
  EXPECT_DOUBLE_EQ(3.0, box->dim.z);
}

TEST(URDF_SCHEMA_GEOMETRY, box_geometry_zero_dimension_allowed_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><box size="1.0 0.0 3.0"/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_FALSE(model->getLink("base")->visual_array.empty());
}

TEST(URDF_SCHEMA_GEOMETRY, box_geometry_no_size_attr_fails)
{
  // parseVisual fails → parseLink returns false → but model still built (link added).
  // The link exists but has no visual.
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_box_no_size.urdf");
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(model->getLink("base")->visual_array.empty());
}

TEST(URDF_SCHEMA_GEOMETRY, sphere_geometry_positive_value)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><sphere radius="0.75"/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_FALSE(link->visual_array.empty());
  auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(link->visual_array[0]->geometry);
  ASSERT_NE(nullptr, sphere);
  EXPECT_DOUBLE_EQ(0.75, sphere->radius);
}

TEST(URDF_SCHEMA_GEOMETRY, sphere_geometry_large_radius_allowed_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><sphere radius="1e6"/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(
    model->getLink("base")->visual_array[0]->geometry);
  ASSERT_NE(nullptr, sphere);
  EXPECT_DOUBLE_EQ(1e6, sphere->radius);
}

TEST(URDF_SCHEMA_GEOMETRY, sphere_geometry_negative_radius_allowed_v1_0)
{
  // v1.0 applies no sign validation on geometry dimensions.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><sphere radius="-0.5"/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(
    model->getLink("base")->visual_array[0]->geometry);
  ASSERT_NE(nullptr, sphere);
  EXPECT_DOUBLE_EQ(-0.5, sphere->radius);
}

TEST(URDF_SCHEMA_GEOMETRY, sphere_geometry_no_radius_attr_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_sphere_no_radius.urdf");
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(model->getLink("base")->visual_array.empty());
}

TEST(URDF_SCHEMA_GEOMETRY, cylinder_geometry_positive_values)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <geometry><cylinder radius="0.4" length="1.2"/></geometry>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(
    model->getLink("base")->visual_array[0]->geometry);
  ASSERT_NE(nullptr, cyl);
  EXPECT_DOUBLE_EQ(0.4, cyl->radius);
  EXPECT_DOUBLE_EQ(1.2, cyl->length);
}

TEST(URDF_SCHEMA_GEOMETRY, cylinder_geometry_negative_radius_allowed_v1_0)
{
  // v1.0 applies no sign validation on geometry dimensions.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><cylinder radius="-0.4" length="1.2"/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(
    model->getLink("base")->visual_array[0]->geometry);
  ASSERT_NE(nullptr, cyl);
  EXPECT_DOUBLE_EQ(-0.4, cyl->radius);
  EXPECT_DOUBLE_EQ( 1.2, cyl->length);
}

TEST(URDF_SCHEMA_GEOMETRY, cylinder_geometry_negative_length_allowed_v1_0)
{
  // v1.0 applies no sign validation on geometry dimensions.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><cylinder radius="0.4" length="-1.2"/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(
    model->getLink("base")->visual_array[0]->geometry);
  ASSERT_NE(nullptr, cyl);
  EXPECT_DOUBLE_EQ( 0.4, cyl->radius);
  EXPECT_DOUBLE_EQ(-1.2, cyl->length);
}

TEST(URDF_SCHEMA_GEOMETRY, cylinder_geometry_no_length_attr_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_cylinder_no_length.urdf");
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(model->getLink("base")->visual_array.empty());
}

TEST(URDF_SCHEMA_GEOMETRY, cylinder_geometry_no_radius_attr_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_cylinder_no_radius.urdf");
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(model->getLink("base")->visual_array.empty());
}

TEST(URDF_SCHEMA_GEOMETRY, mesh_geometry_with_filename)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <geometry>
            <mesh filename="package://my_pkg/meshes/base.dae"/>
          </geometry>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(
    model->getLink("base")->visual_array[0]->geometry);
  ASSERT_NE(nullptr, mesh);
  EXPECT_EQ("package://my_pkg/meshes/base.dae", mesh->filename);
}

TEST(URDF_SCHEMA_GEOMETRY, mesh_geometry_default_scale_is_one)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <geometry><mesh filename="my_mesh.stl"/></geometry>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(
    model->getLink("base")->visual_array[0]->geometry);
  ASSERT_NE(nullptr, mesh);
  EXPECT_DOUBLE_EQ(1.0, mesh->scale.x);
  EXPECT_DOUBLE_EQ(1.0, mesh->scale.y);
  EXPECT_DOUBLE_EQ(1.0, mesh->scale.z);
}

TEST(URDF_SCHEMA_GEOMETRY, mesh_geometry_with_explicit_scale)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <geometry>
            <mesh filename="my_mesh.stl" scale="0.001 0.001 0.001"/>
          </geometry>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(
    model->getLink("base")->visual_array[0]->geometry);
  ASSERT_NE(nullptr, mesh);
  EXPECT_DOUBLE_EQ(0.001, mesh->scale.x);
  EXPECT_DOUBLE_EQ(0.001, mesh->scale.y);
  EXPECT_DOUBLE_EQ(0.001, mesh->scale.z);
}

TEST(URDF_SCHEMA_GEOMETRY, mesh_geometry_no_filename_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_mesh_no_filename.urdf");
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(model->getLink("base")->visual_array.empty());
}

TEST(URDF_SCHEMA_GEOMETRY, unknown_geometry_type_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_unknown_geometry_type.urdf");
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(model->getLink("base")->visual_array.empty());
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  setlocale(LC_ALL, "");
  return RUN_ALL_TESTS();
}
