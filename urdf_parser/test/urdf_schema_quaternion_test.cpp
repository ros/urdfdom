#include <gtest/gtest.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

#include "urdf_model/pose.h"
#include "urdf_parser/urdf_parser.h"

TEST(URDF_UNIT_TEST, parse_rot_rpy_version_1_0)
{
  std::string joint_str = R"urdf(
    <robot name="rot_rpy" version="1.0">
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <origin xyz="0 0 0" rpy="1.5707963 0 1.5707963"/>
      </joint>
      <joint name="j2" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
      </joint>
      <link name="l1"/>
      <link name="l2"/>
    </robot>
    )urdf";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(joint_str);

  EXPECT_EQ(2u, urdf->links_.size());
  EXPECT_EQ(2u, urdf->joints_.size());
  EXPECT_EQ("rot_rpy", urdf->name_);

  const urdf::Rotation &j1_rot = urdf->joints_["j1"]->parent_to_joint_origin_transform.rotation;
  double x, y, z, w;
  j1_rot.getQuaternion(x, y, z, w);

  const double tol = 1e-6;
  EXPECT_NEAR(0.5, x, tol);
  EXPECT_NEAR(0.5, y, tol);
  EXPECT_NEAR(0.5, z, tol);
  EXPECT_NEAR(0.5, w, tol);
}

TEST(URDF_UNIT_TEST, parse_rot_quat_ignored_version_1_0)
{
  std::string joint_str = R"urdf(
    <robot name="rot_quat_ignored" version="1.0">
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <origin xyz="0 0 0" quat_xyzw="0.5 0.5 0.5 0.5"/>
      </joint>
      <joint name="j2" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
      </joint>
      <link name="l1"/>
      <link name="l2"/>
    </robot>
    )urdf";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(joint_str);

  EXPECT_EQ(2u, urdf->links_.size());
  EXPECT_EQ(2u, urdf->joints_.size());
  EXPECT_EQ("rot_quat_ignored", urdf->name_);

  const urdf::Rotation &j1_rot = urdf->joints_["j1"]->parent_to_joint_origin_transform.rotation;
  double x, y, z, w;
  j1_rot.getQuaternion(x, y, z, w);

  // the quaternion string is ignored since the version is 1.0
  // expect identity rotation
  EXPECT_DOUBLE_EQ(0.0, x);
  EXPECT_DOUBLE_EQ(0.0, y);
  EXPECT_DOUBLE_EQ(0.0, z);
  EXPECT_DOUBLE_EQ(1.0, w);
}

TEST(URDF_UNIT_TEST, parse_rot_quat_version_1_1)
{
  std::string joint_str = R"urdf(
    <robot name="rot_quat" version="1.1">
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <origin xyz="0 0 0" quat_xyzw="0.5 0.5 0.5 0.5"/>
      </joint>
      <joint name="j2" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
      </joint>
      <link name="l1"/>
      <link name="l2"/>
    </robot>
    )urdf";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(joint_str);

  EXPECT_EQ(2u, urdf->links_.size());
  EXPECT_EQ(2u, urdf->joints_.size());
  EXPECT_EQ("rot_quat", urdf->name_);

  const urdf::Rotation &j1_rot = urdf->joints_["j1"]->parent_to_joint_origin_transform.rotation;
  double x, y, z, w;
  j1_rot.getQuaternion(x, y, z, w);

  EXPECT_DOUBLE_EQ(0.5, x);
  EXPECT_DOUBLE_EQ(0.5, y);
  EXPECT_DOUBLE_EQ(0.5, z);
  EXPECT_DOUBLE_EQ(0.5, w);
}

TEST(URDF_UNIT_TEST, parse_rot_both_quat_ignored_version_1_0)
{
  std::string joint_str = R"urdf(
    <robot name="rot_both_quat_ignored" version="1.0">
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <origin xyz="0 0 0" rpy="0 0 0" quat_xyzw="0.5 0.5 0.5 0.5"/>
      </joint>
      <joint name="j2" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
      </joint>
      <link name="l1"/>
      <link name="l2"/>
    </robot>
    )urdf";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(joint_str);

  EXPECT_EQ(2u, urdf->links_.size());
  EXPECT_EQ(2u, urdf->joints_.size());
  EXPECT_EQ("rot_both_quat_ignored", urdf->name_);

  const urdf::Rotation &j1_rot = urdf->joints_["j1"]->parent_to_joint_origin_transform.rotation;
  double x, y, z, w;
  j1_rot.getQuaternion(x, y, z, w);

  // the quaternion string is ignored since the version is 1.0
  // expect identity rotation
  EXPECT_DOUBLE_EQ(0.0, x);
  EXPECT_DOUBLE_EQ(0.0, y);
  EXPECT_DOUBLE_EQ(0.0, z);
  EXPECT_DOUBLE_EQ(1.0, w);
}

TEST(URDF_UNIT_TEST, parse_rot_both_error_version_1_1)
{
  std::string joint_str = R"urdf(
    <robot name="rot_both_error" version="1.1">
      <joint name="j1" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
        <origin xyz="0 0 0" rpy="0 0 0" quat_xyzw="0.5 0.5 0.5 0.5"/>
      </joint>
      <joint name="j2" type="fixed">
        <parent link="l1"/>
        <child link="l2"/>
      </joint>
      <link name="l1"/>
      <link name="l2"/>
    </robot>
    )urdf";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(joint_str);
  EXPECT_EQ(nullptr, urdf);
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // use the environment locale so that the unit test can be repeated with various locales easily
  setlocale(LC_ALL, "");

  return RUN_ALL_TESTS();
}
