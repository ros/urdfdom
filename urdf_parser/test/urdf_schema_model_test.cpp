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

TEST(URDF_SCHEMA_MODEL, no_version_attr_defaults_to_v1_0)
{
  // When version is omitted the parser defaults to 1.0.
  std::string urdf_str = R"urdf(
    <robot name="no_version">
      <link name="base"/>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("no_version", model->name_);
  EXPECT_EQ(1u, model->links_.size());
}

TEST(URDF_SCHEMA_MODEL, single_link_robot_is_valid)
{
  std::string urdf_str = R"urdf(
    <robot name="single" version="1.0">
      <link name="only_link"/>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ(1u, model->links_.size());
  ASSERT_NE(nullptr, model->root_link_);
  EXPECT_EQ("only_link", model->root_link_->name);
}

TEST(URDF_SCHEMA_MODEL, invalid_xml_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_invalid_xml.urdf");
  EXPECT_EQ(nullptr, model);
}

TEST(URDF_SCHEMA_MODEL, no_robot_element_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_no_robot_element.urdf");
  EXPECT_EQ(nullptr, model);
}

TEST(URDF_SCHEMA_MODEL, robot_without_name_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_robot_no_name.urdf");
  EXPECT_EQ(nullptr, model);
}

TEST(URDF_SCHEMA_MODEL, no_links_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_no_links.urdf");
  EXPECT_EQ(nullptr, model);
}

TEST(URDF_SCHEMA_MODEL, unsupported_version_too_high_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_version_too_high.urdf");
  EXPECT_EQ(nullptr, model);
}

TEST(URDF_SCHEMA_MODEL, unsupported_version_too_low_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_version_too_low.urdf");
  EXPECT_EQ(nullptr, model);
}

TEST(URDF_SCHEMA_MODEL, malformed_version_string_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_malformed_version.urdf");
  EXPECT_EQ(nullptr, model);
}

TEST(URDF_SCHEMA_MODEL, duplicate_link_name_fails)
{
  EXPECT_EQ(nullptr, urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_duplicate_link_name.urdf"));
}

TEST(URDF_SCHEMA_MODEL, duplicate_joint_name_fails)
{
  EXPECT_EQ(nullptr, urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_duplicate_joint_name.urdf"));
}

TEST(URDF_SCHEMA_MODEL, joint_referencing_unknown_parent_link_fails)
{
  EXPECT_EQ(nullptr, urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_joint_unknown_parent.urdf"));
}

TEST(URDF_SCHEMA_MODEL, joint_referencing_unknown_child_link_fails)
{
  EXPECT_EQ(nullptr, urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_joint_unknown_child.urdf"));
}

TEST(URDF_SCHEMA_MODEL, two_root_links_fails)
{
  // Two disconnected links with no joint → two root links → initRoot fails.
  EXPECT_EQ(nullptr, urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/bad_two_root_links.urdf"));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  setlocale(LC_ALL, "");
  return RUN_ALL_TESTS();
}
