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
#include <string>

#include "urdf_model/joint.h"
#include "urdf_model/link.h"
#include "urdf_parser/urdf_parser.h"

TEST(URDF_SCHEMA_FULL_ROBOT, full_v1_0_robot_all_features)
{
  // Exercises: all geometry types, inertial, materials, every joint type,
  // dynamics, safety, calibration, mimic, origins with xyz+rpy, multiple
  // visuals, multiple collisions, global + inline materials.
  // See test/assets/full_robot_v1_0.urdf for the URDF source.
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(
    std::string(TEST_ASSETS_DIR) + "/full_robot_v1_0.urdf");
  ASSERT_NE(nullptr, model);

  EXPECT_EQ("full_robot", model->name_);
  EXPECT_EQ(7u, model->links_.size());
  EXPECT_EQ(6u, model->joints_.size());
  ASSERT_NE(nullptr, model->root_link_);
  EXPECT_EQ("base_link", model->root_link_->name);

  // Verify global materials
  ASSERT_NE(nullptr, model->getMaterial("steel"));
  ASSERT_NE(nullptr, model->getMaterial("rubber"));

  // base_link inertial
  auto base = model->getLink("base_link");
  ASSERT_NE(nullptr, base->inertial);
  EXPECT_DOUBLE_EQ(10.0, base->inertial->mass);
  EXPECT_DOUBLE_EQ(0.1,  base->inertial->ixx);
  EXPECT_EQ(1u, base->visual_array.size());
  EXPECT_EQ(1u, base->collision_array.size());
  EXPECT_EQ(urdf::Geometry::BOX, base->visual->geometry->type);

  // shoulder joint
  auto sj = model->getJoint("shoulder_joint");
  ASSERT_NE(nullptr, sj);
  EXPECT_EQ(urdf::Joint::REVOLUTE, sj->type);
  ASSERT_NE(nullptr, sj->limits);
  EXPECT_DOUBLE_EQ(-1.5707963, sj->limits->lower);
  EXPECT_DOUBLE_EQ( 1.5707963, sj->limits->upper);
  EXPECT_DOUBLE_EQ(50.0, sj->limits->effort);
  EXPECT_DOUBLE_EQ( 1.0, sj->limits->velocity);
  ASSERT_NE(nullptr, sj->dynamics);
  EXPECT_DOUBLE_EQ(1.0, sj->dynamics->damping);
  EXPECT_DOUBLE_EQ(0.1, sj->dynamics->friction);
  ASSERT_NE(nullptr, sj->safety);
  EXPECT_DOUBLE_EQ(10.0, sj->safety->k_position);
  EXPECT_DOUBLE_EQ( 5.0, sj->safety->k_velocity);
  ASSERT_NE(nullptr, sj->calibration->rising);
  EXPECT_DOUBLE_EQ(0.0, *sj->calibration->rising);
  ASSERT_NE(nullptr, sj->calibration->falling);
  EXPECT_DOUBLE_EQ(-0.1, *sj->calibration->falling);
  EXPECT_TRUE(std::isinf(sj->limits->acceleration));
  EXPECT_TRUE(std::isinf(sj->limits->deceleration));
  EXPECT_TRUE(std::isinf(sj->limits->jerk));

  // elbow joint (prismatic)
  auto ej = model->getJoint("elbow_joint");
  ASSERT_NE(nullptr, ej);
  EXPECT_EQ(urdf::Joint::PRISMATIC, ej->type);

  // wrist joint (continuous — no limits)
  auto wj = model->getJoint("wrist_joint");
  ASSERT_NE(nullptr, wj);
  EXPECT_EQ(urdf::Joint::CONTINUOUS, wj->type);
  EXPECT_EQ(nullptr, wj->limits);

  // wrist link has mesh visual and sphere collision
  auto wrist = model->getLink("wrist_link");
  ASSERT_NE(nullptr, wrist);
  ASSERT_FALSE(wrist->visual_array.empty());
  EXPECT_EQ(urdf::Geometry::MESH, wrist->visual->geometry->type);
  auto wrist_mesh = std::dynamic_pointer_cast<urdf::Mesh>(wrist->visual->geometry);
  EXPECT_EQ("package://robot/meshes/wrist.dae", wrist_mesh->filename);

  // tool joint (fixed)
  EXPECT_EQ(urdf::Joint::FIXED, model->getJoint("tool_joint")->type);

  // camera joint (floating)
  EXPECT_EQ(urdf::Joint::FLOATING, model->getJoint("camera_joint")->type);

  // platform joint (planar) + mimic
  auto pj = model->getJoint("platform_joint");
  ASSERT_NE(nullptr, pj);
  EXPECT_EQ(urdf::Joint::PLANAR, pj->type);
  ASSERT_NE(nullptr, pj->mimic);
  EXPECT_EQ("shoulder_joint", pj->mimic->joint_name);
  EXPECT_DOUBLE_EQ(0.5, pj->mimic->multiplier);
  EXPECT_DOUBLE_EQ(0.0, pj->mimic->offset);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  setlocale(LC_ALL, "");
  return RUN_ALL_TESTS();
}
