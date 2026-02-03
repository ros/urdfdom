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
#include <memory>

#include "urdf_model/link.h"
#include "urdf_parser/urdf_parser.h"

TEST(URDF_UNIT_TEST, parse_capsule_geometry_version_1_1)
{
  std::string urdf_str = R"urdf(
    <robot name="capsule_test" version="1.1">
      <link name="link1">
        <visual>
          <geometry>
            <capsule radius="0.05" length="0.5"/>
          </geometry>
        </visual>
        <collision>
          <geometry>
            <capsule radius="0.1" length="1.0"/>
          </geometry>
        </collision>
      </link>
    </robot>
    )urdf";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(urdf_str);

  ASSERT_NE(nullptr, urdf);
  EXPECT_EQ("capsule_test", urdf->name_);
  EXPECT_EQ(1u, urdf->links_.size());

  urdf::LinkConstSharedPtr link = urdf->getLink("link1");
  ASSERT_NE(nullptr, link);

  // Check visual geometry
  ASSERT_EQ(1u, link->visual_array.size());
  ASSERT_NE(nullptr, link->visual_array[0]->geometry);
  EXPECT_EQ(urdf::Geometry::CAPSULE, link->visual_array[0]->geometry->type);

  std::shared_ptr<urdf::Capsule> visual_capsule =
      std::dynamic_pointer_cast<urdf::Capsule>(link->visual_array[0]->geometry);
  ASSERT_NE(nullptr, visual_capsule);
  EXPECT_DOUBLE_EQ(0.05, visual_capsule->radius);
  EXPECT_DOUBLE_EQ(0.5, visual_capsule->length);

  // Check collision geometry
  ASSERT_EQ(1u, link->collision_array.size());
  ASSERT_NE(nullptr, link->collision_array[0]->geometry);
  EXPECT_EQ(urdf::Geometry::CAPSULE, link->collision_array[0]->geometry->type);

  std::shared_ptr<urdf::Capsule> collision_capsule =
      std::dynamic_pointer_cast<urdf::Capsule>(link->collision_array[0]->geometry);
  ASSERT_NE(nullptr, collision_capsule);
  EXPECT_DOUBLE_EQ(0.1, collision_capsule->radius);
  EXPECT_DOUBLE_EQ(1.0, collision_capsule->length);
}

TEST(URDF_UNIT_TEST, parse_capsule_geometry_ignored_version_1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="capsule_ignored_test" version="1.0">
      <link name="link1">
        <visual>
          <geometry>
            <capsule radius="0.05" length="0.5"/>
          </geometry>
        </visual>
      </link>
    </robot>
    )urdf";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(urdf_str);

  // Capsule is not supported in version 1.0 and is ignored with a warning.
  // The link is still created, but visual parsing fails.
  ASSERT_NE(nullptr, urdf);
  EXPECT_EQ("capsule_ignored_test", urdf->name_);

  urdf::LinkConstSharedPtr link = urdf->getLink("link1");
  ASSERT_NE(nullptr, link);
  // Visual should be empty since capsule geometry was ignored
  EXPECT_TRUE(link->visual_array.empty());
}

TEST(URDF_UNIT_TEST, parse_capsule_geometry_zero_values)
{
  std::string urdf_str = R"urdf(
    <robot name="capsule_zero_test" version="1.1">
      <link name="link1">
        <visual>
          <geometry>
            <capsule radius="0.0" length="0.0"/>
          </geometry>
        </visual>
      </link>
    </robot>
    )urdf";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(urdf_str);

  ASSERT_NE(nullptr, urdf);
  urdf::LinkConstSharedPtr link = urdf->getLink("link1");
  ASSERT_NE(nullptr, link);

  std::shared_ptr<urdf::Capsule> capsule =
      std::dynamic_pointer_cast<urdf::Capsule>(link->visual_array[0]->geometry);
  ASSERT_NE(nullptr, capsule);
  EXPECT_DOUBLE_EQ(0.0, capsule->radius);
  EXPECT_DOUBLE_EQ(0.0, capsule->length);
}

TEST(URDF_UNIT_TEST, parse_capsule_geometry_missing_radius_fails)
{
  std::string urdf_str = R"urdf(
    <robot name="capsule_missing_radius_test" version="1.1">
      <link name="link1">
        <visual>
          <geometry>
            <capsule length="0.5"/>
          </geometry>
        </visual>
      </link>
    </robot>
    )urdf";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(urdf_str);
  // Missing radius causes geometry parsing to fail, visual is empty
  ASSERT_NE(nullptr, urdf);
  urdf::LinkConstSharedPtr link = urdf->getLink("link1");
  ASSERT_NE(nullptr, link);
  EXPECT_TRUE(link->visual_array.empty());
}

TEST(URDF_UNIT_TEST, parse_capsule_geometry_missing_length_fails)
{
  std::string urdf_str = R"urdf(
    <robot name="capsule_missing_length_test" version="1.1">
      <link name="link1">
        <visual>
          <geometry>
            <capsule radius="0.05"/>
          </geometry>
        </visual>
      </link>
    </robot>
    )urdf";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(urdf_str);
  // Missing length causes geometry parsing to fail, visual is empty
  ASSERT_NE(nullptr, urdf);
  urdf::LinkConstSharedPtr link = urdf->getLink("link1");
  ASSERT_NE(nullptr, link);
  EXPECT_TRUE(link->visual_array.empty());
}

TEST(URDF_UNIT_TEST, parse_capsule_geometry_inf_value_fails)
{
  std::string urdf_str = R"urdf(
    <robot name="capsule_inf_test" version="1.1">
      <link name="link1">
        <visual>
          <geometry>
            <capsule radius="inf" length="0.5"/>
          </geometry>
        </visual>
      </link>
    </robot>
    )urdf";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(urdf_str);
  // Infinity value causes geometry parsing to fail, visual is empty
  ASSERT_NE(nullptr, urdf);
  urdf::LinkConstSharedPtr link = urdf->getLink("link1");
  ASSERT_NE(nullptr, link);
  EXPECT_TRUE(link->visual_array.empty());
}

TEST(URDF_UNIT_TEST, parse_multiple_capsules_in_same_link)
{
  std::string urdf_str = R"urdf(
    <robot name="multi_capsule_test" version="1.1">
      <link name="link1">
        <visual>
          <geometry>
            <capsule radius="0.05" length="0.5"/>
          </geometry>
        </visual>
        <visual>
          <geometry>
            <capsule radius="0.1" length="1.0"/>
          </geometry>
        </visual>
        <collision>
          <geometry>
            <capsule radius="0.15" length="1.5"/>
          </geometry>
        </collision>
        <collision>
          <geometry>
            <capsule radius="0.2" length="2.0"/>
          </geometry>
        </collision>
      </link>
    </robot>
    )urdf";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(urdf_str);

  ASSERT_NE(nullptr, urdf);
  EXPECT_EQ("multi_capsule_test", urdf->name_);

  urdf::LinkConstSharedPtr link = urdf->getLink("link1");
  ASSERT_NE(nullptr, link);

  // Check multiple visual geometries
  ASSERT_EQ(2u, link->visual_array.size());

  ASSERT_NE(nullptr, link->visual_array[0]->geometry);
  EXPECT_EQ(urdf::Geometry::CAPSULE, link->visual_array[0]->geometry->type);
  std::shared_ptr<urdf::Capsule> visual_capsule_0 =
      std::dynamic_pointer_cast<urdf::Capsule>(link->visual_array[0]->geometry);
  ASSERT_NE(nullptr, visual_capsule_0);
  EXPECT_DOUBLE_EQ(0.05, visual_capsule_0->radius);
  EXPECT_DOUBLE_EQ(0.5, visual_capsule_0->length);

  ASSERT_NE(nullptr, link->visual_array[1]->geometry);
  EXPECT_EQ(urdf::Geometry::CAPSULE, link->visual_array[1]->geometry->type);
  std::shared_ptr<urdf::Capsule> visual_capsule_1 =
      std::dynamic_pointer_cast<urdf::Capsule>(link->visual_array[1]->geometry);
  ASSERT_NE(nullptr, visual_capsule_1);
  EXPECT_DOUBLE_EQ(0.1, visual_capsule_1->radius);
  EXPECT_DOUBLE_EQ(1.0, visual_capsule_1->length);

  // Check multiple collision geometries
  ASSERT_EQ(2u, link->collision_array.size());

  ASSERT_NE(nullptr, link->collision_array[0]->geometry);
  EXPECT_EQ(urdf::Geometry::CAPSULE, link->collision_array[0]->geometry->type);
  std::shared_ptr<urdf::Capsule> collision_capsule_0 =
      std::dynamic_pointer_cast<urdf::Capsule>(link->collision_array[0]->geometry);
  ASSERT_NE(nullptr, collision_capsule_0);
  EXPECT_DOUBLE_EQ(0.15, collision_capsule_0->radius);
  EXPECT_DOUBLE_EQ(1.5, collision_capsule_0->length);

  ASSERT_NE(nullptr, link->collision_array[1]->geometry);
  EXPECT_EQ(urdf::Geometry::CAPSULE, link->collision_array[1]->geometry->type);
  std::shared_ptr<urdf::Capsule> collision_capsule_1 =
      std::dynamic_pointer_cast<urdf::Capsule>(link->collision_array[1]->geometry);
  ASSERT_NE(nullptr, collision_capsule_1);
  EXPECT_DOUBLE_EQ(0.2, collision_capsule_1->radius);
  EXPECT_DOUBLE_EQ(2.0, collision_capsule_1->length);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // use the environment locale so that the unit test can be repeated with various locales easily
  setlocale(LC_ALL, "");

  return RUN_ALL_TESTS();
}

