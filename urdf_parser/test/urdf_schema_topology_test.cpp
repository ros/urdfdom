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

TEST(URDF_SCHEMA_TOPOLOGY, three_link_chain_topology)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/three_link_chain.urdf");
  ASSERT_NE(nullptr, model);
  EXPECT_EQ(3u, model->links_.size());
  EXPECT_EQ(2u, model->joints_.size());
  ASSERT_NE(nullptr, model->root_link_);
  EXPECT_EQ("base", model->root_link_->name);

  auto base = model->getLink("base");
  ASSERT_NE(nullptr, base);
  EXPECT_EQ(1u, base->child_links.size());
  EXPECT_EQ("mid", base->child_links[0]->name);

  auto mid = model->getLink("mid");
  ASSERT_NE(nullptr, mid);
  EXPECT_EQ(1u, mid->child_links.size());
  EXPECT_EQ("tip", mid->child_links[0]->name);

  auto tip = model->getLink("tip");
  ASSERT_NE(nullptr, tip);
  EXPECT_EQ(0u, tip->child_links.size());
}

TEST(URDF_SCHEMA_TOPOLOGY, branching_topology)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/branching_robot.urdf");
  ASSERT_NE(nullptr, model);
  EXPECT_EQ(4u, model->links_.size());
  EXPECT_EQ(3u, model->joints_.size());
  ASSERT_NE(nullptr, model->root_link_);
  EXPECT_EQ("torso", model->root_link_->name);
  EXPECT_EQ(3u, model->getLink("torso")->child_links.size());
}

TEST(URDF_SCHEMA_TOPOLOGY, parent_link_tree_is_correct)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base"/>
      <link name="child"/>
      <joint name="j1" type="fixed">
        <parent link="base"/>
        <child link="child"/>
      </joint>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto child = model->getLink("child");
  ASSERT_NE(nullptr, child);
  ASSERT_NE(nullptr, child->getParent());
  EXPECT_EQ("base", child->getParent()->name);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  setlocale(LC_ALL, "");
  return RUN_ALL_TESTS();
}
