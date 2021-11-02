#include <gtest/gtest.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

#include "urdf_model/pose.h"
#include "urdf_parser/urdf_parser.h"

#ifndef M_PI
  # define M_PI 3.141592653589793
#endif

bool quat_are_near(urdf::Rotation left, urdf::Rotation right)
{
  static const double epsilon = 1e-3; // quite loose epsilon
  double l[4], r[4];
  left.getQuaternion(l[0], l[1], l[2], l[3]);
  right.getQuaternion(r[0], r[1], r[2], r[3]);
  return (std::abs(l[0] - r[0]) < epsilon &&
          std::abs(l[1] - r[1]) < epsilon &&
          std::abs(l[2] - r[2]) < epsilon &&
          std::abs(l[3] - r[3]) < epsilon) ||
         (std::abs(l[0] + r[0]) < epsilon &&
          std::abs(l[1] + r[1]) < epsilon &&
          std::abs(l[2] + r[2]) < epsilon &&
          std::abs(l[3] + r[3]) < epsilon);
}

std::ostream &operator<<(std::ostream &os, const urdf::Rotation& rot)
{
  double roll, pitch, yaw;
  double x, y, z, w;
  rot.getRPY(roll, pitch, yaw);
  rot.getQuaternion(x, y, z, w);
  os << std::setprecision(9)
     << "x: " << x << " y: " << y << " z: " << z << " w: " <<  w
     << "  roll: "  << roll << " pitch: " << pitch << " yaw: "<< yaw;
  return os;
}


void check_get_set_rpy_is_idempotent(double x, double y, double z, double w)
{
  urdf::Rotation rot0;
  rot0.setFromQuaternion(x, y, z, w);
  double roll, pitch, yaw;
  rot0.getRPY(roll, pitch, yaw);
  urdf::Rotation rot1;
  rot1.setFromRPY(roll, pitch, yaw);
  if (true) {
    std::cout << "\n"
              << "before  " << rot0 << "\n"
              << "after   " << rot1 << "\n"
              << "ok      " << quat_are_near(rot0, rot1) << "\n";
  }
  EXPECT_TRUE(quat_are_near(rot0, rot1));
}

void check_get_set_rpy_is_idempotent_from_rpy(double r, double p, double y)
{
  urdf::Rotation rot0;
  rot0.setFromRPY(r, p, y);
  double roll, pitch, yaw;
  rot0.getRPY(roll, pitch, yaw);
  urdf::Rotation rot1;
  rot1.setFromRPY(roll, pitch, yaw);
  bool ok = quat_are_near(rot0, rot1);
  if (!ok) {
    std::cout << "initial rpy: " << r << " " << p << " " << y << "\n"
              << "before  " << rot0 << "\n"
              << "after   " << rot1 << "\n"
              << "ok      " << ok << "\n";
  }
  EXPECT_TRUE(ok);
}

TEST(URDF_UNIT_TEST, test_rotation_get_set_rpy_idempotent)
{
  double x0 = 0.5, y0 = -0.5, z0 = 0.5,  w0 = 0.5;
  check_get_set_rpy_is_idempotent(x0, y0, z0, w0);
  double delta = 2.2e-8;
  check_get_set_rpy_is_idempotent(x0, y0, z0+delta, w0-delta);

  // Checking consistency (in quaternion space) of set/get rpy
  check_get_set_rpy_is_idempotent_from_rpy(0.0,-M_PI/2,0.0);


  // More complete consistency check of set/get rpy
  // We define a list of angles (some totally random,
  // some instead are cornercase such as 0.0 or M_PI).
  // Then we check the consistency for all possible
  // permutations with repetition (nrOfAngles^3)
  std::vector<double> testAngles;
  testAngles.push_back(0.0);
  testAngles.push_back(M_PI/4);
  testAngles.push_back(M_PI/3);
  testAngles.push_back(M_PI/2);
  testAngles.push_back(M_PI);
  testAngles.push_back(-M_PI/4);
  testAngles.push_back(-M_PI/3);
  testAngles.push_back(-M_PI/2);
  testAngles.push_back(-M_PI);
  testAngles.push_back(1.0);
  testAngles.push_back(1.5);
  testAngles.push_back(2.0);
  testAngles.push_back(-1.0);
  testAngles.push_back(-1.5);
  testAngles.push_back(-2.0);

  for(size_t rIdx = 0; rIdx < testAngles.size(); rIdx++ ) {
    for(size_t pIdx = 0; pIdx < testAngles.size(); pIdx++ ) {
      for(size_t yIdx = 0; yIdx < testAngles.size(); yIdx++ ) {
            check_get_set_rpy_is_idempotent_from_rpy(testAngles[rIdx],
                                                     testAngles[pIdx],
                                                     testAngles[yIdx]);
      }
    }
  }
}

TEST(URDF_UNIT_TEST, test_vector3_simple)
{
  urdf::Vector3 vec;

  vec.init("1.0 2.0 3.0");

  EXPECT_EQ(1.0, vec.x);
  EXPECT_EQ(2.0, vec.y);
  EXPECT_EQ(3.0, vec.z);
}

TEST(URDF_UNIT_TEST, test_vector3_float)
{
  urdf::Vector3 vec;

  vec.init("0.1 0.2 0.3");

  EXPECT_EQ(0.1, vec.x);
  EXPECT_EQ(0.2, vec.y);
  EXPECT_EQ(0.3, vec.z);
}

TEST(URDF_UNIT_TEST, test_vector3_bad_string)
{
  urdf::Vector3 vec;

  EXPECT_THROW(vec.init("1.0 foo 3.0"), urdf::ParseError);
}

TEST(URDF_UNIT_TEST, test_vector3_invalid_number)
{
  urdf::Vector3 vec;

  EXPECT_THROW(vec.init("1.0 2.10.110 3.0"), urdf::ParseError);
}

TEST(URDF_UNIT_TEST, test_vector3_not_enough_numbers)
{
  urdf::Vector3 vec;

  EXPECT_THROW(vec.init("1.0 2.0"), urdf::ParseError);
}

TEST(URDF_UNIT_TEST, test_vector3_too_many_numbers)
{
  urdf::Vector3 vec;

  EXPECT_THROW(vec.init("1.0 2.0 3.0 4.0"), urdf::ParseError);
}

TEST(URDF_UNIT_TEST, parse_joint_doubles)
{
  std::string joint_str =
    "<robot name=\"test\">"
    "  <joint name=\"j1\" type=\"fixed\">"
    "    <parent link=\"l1\"/>"
    "    <child link=\"l2\"/>"
    "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
    "    <dynamics damping=\"87.098\" friction=\"3.1290\"/>"
    "    <limit lower=\"12.34\" upper=\"22.999\" effort=\"99.0\" velocity=\"23.0\"/>"
    "    <safety_controller soft_lower_limit=\"8.765\" soft_upper_limit=\"9.003\" k_position=\"7.0034\" k_velocity=\"9.998\"/>"
    "    <calibration rising=\"8.654\" falling=\"0.0445\"/>"
    "    <mimic joint=\"j2\" multiplier=\"9.87\" offset=\"0.098\"/>"
    "  </joint>"
    "  <joint name=\"j2\" type=\"fixed\">"
    "    <parent link=\"l1\"/>"
    "    <child link=\"l2\"/>"
    "  </joint>"
    "  <link name=\"l1\"/>"
    "  <link name=\"l2\"/>"
    "</robot>";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(joint_str);

  EXPECT_EQ(2u, urdf->links_.size());
  EXPECT_EQ(2u, urdf->joints_.size());
  EXPECT_EQ("test", urdf->name_);

  EXPECT_EQ(87.098, urdf->joints_["j1"]->dynamics->damping);
  EXPECT_EQ(3.1290, urdf->joints_["j1"]->dynamics->friction);

  EXPECT_EQ(12.34, urdf->joints_["j1"]->limits->lower);
  EXPECT_EQ(22.999, urdf->joints_["j1"]->limits->upper);
  EXPECT_EQ(99.0, urdf->joints_["j1"]->limits->effort);
  EXPECT_EQ(23.0, urdf->joints_["j1"]->limits->velocity);

  EXPECT_EQ(8.765, urdf->joints_["j1"]->safety->soft_lower_limit);
  EXPECT_EQ(9.003, urdf->joints_["j1"]->safety->soft_upper_limit);
  EXPECT_EQ(7.0034, urdf->joints_["j1"]->safety->k_position);
  EXPECT_EQ(9.998, urdf->joints_["j1"]->safety->k_velocity);

  EXPECT_EQ(8.654, *urdf->joints_["j1"]->calibration->rising);
  EXPECT_EQ(0.0445, *urdf->joints_["j1"]->calibration->falling);

  EXPECT_EQ(9.87, urdf->joints_["j1"]->mimic->multiplier);
  EXPECT_EQ(0.098, urdf->joints_["j1"]->mimic->offset);
}

TEST(URDF_UNIT_TEST, parse_link_doubles)
{
  std::string joint_str =
    "<robot name=\"test\">"
    "  <joint name=\"j1\" type=\"fixed\">"
    "    <parent link=\"l1\"/>"
    "    <child link=\"l2\"/>"
    "  </joint>"
    "  <joint name=\"j2\" type=\"fixed\">"
    "    <parent link=\"l1\"/>"
    "    <child link=\"l2\"/>"
    "  </joint>"
    "  <link name=\"l1\">"
    "    <visual>"
    "      <geometry>"
    "        <sphere radius=\"1.349\"/>"
    "      </geometry>"
    "    </visual>"
    "    <inertial>"
    "      <mass value=\"8.4396\"/>"
    "      <inertia ixx=\"0.087\" ixy=\"0.14\" ixz=\"0.912\" iyy=\"0.763\" iyz=\"0.0012\" izz=\"0.908\"/>"
    "    </inertial>"
    "  </link>"
    "  <link name=\"l2\">"
    "    <visual>"
    "      <geometry>"
    "        <cylinder radius=\"3.349\" length=\"7.5490\"/>"
    "      </geometry>"
    "    </visual>"
    "  </link>"
    "</robot>";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(joint_str);

  EXPECT_EQ(2u, urdf->links_.size());
  EXPECT_EQ(2u, urdf->joints_.size());

  EXPECT_EQ(urdf::Geometry::SPHERE, urdf->links_["l1"]->visual->geometry->type);
  std::shared_ptr<urdf::Sphere> s = std::dynamic_pointer_cast<urdf::Sphere>(urdf->links_["l1"]->visual->geometry);
  EXPECT_EQ(1.349, s->radius);

  EXPECT_EQ(urdf::Geometry::CYLINDER, urdf->links_["l2"]->visual->geometry->type);
  std::shared_ptr<urdf::Cylinder> c = std::dynamic_pointer_cast<urdf::Cylinder>(urdf->links_["l2"]->visual->geometry);
  EXPECT_EQ(3.349, c->radius);
  EXPECT_EQ(7.5490, c->length);

  EXPECT_EQ(8.4396, urdf->links_["l1"]->inertial->mass);
  EXPECT_EQ(0.087, urdf->links_["l1"]->inertial->ixx);
  EXPECT_EQ(0.14, urdf->links_["l1"]->inertial->ixy);
  EXPECT_EQ(0.912, urdf->links_["l1"]->inertial->ixz);
  EXPECT_EQ(0.763, urdf->links_["l1"]->inertial->iyy);
  EXPECT_EQ(0.0012, urdf->links_["l1"]->inertial->iyz);
  EXPECT_EQ(0.908, urdf->links_["l1"]->inertial->izz);
}


TEST(URDF_UNIT_TEST, parse_color_doubles)
{
  std::string joint_str =
    "<robot name=\"test\">"
    "  <joint name=\"j1\" type=\"fixed\">"
    "    <parent link=\"l1\"/>"
    "    <child link=\"l2\"/>"
    "  </joint>"
    "  <joint name=\"j2\" type=\"fixed\">"
    "    <parent link=\"l1\"/>"
    "    <child link=\"l2\"/>"
    "  </joint>"
    "  <link name=\"l1\">"
    "    <visual>"
    "      <geometry>"
    "        <sphere radius=\"1.349\"/>"
    "      </geometry>"
    "      <material name=\"\">"
    "        <color rgba=\"1.0 0.65 0.0 0.01\" />"
    "      </material>"
    "    </visual>"
    "    <inertial>"
    "      <mass value=\"8.4396\"/>"
    "      <inertia ixx=\"0.087\" ixy=\"0.14\" ixz=\"0.912\" iyy=\"0.763\" iyz=\"0.0012\" izz=\"0.908\"/>"
    "    </inertial>"
    "  </link>"
    "  <link name=\"l2\">"
    "    <visual>"
    "      <geometry>"
    "        <cylinder radius=\"3.349\" length=\"7.5490\"/>"
    "      </geometry>"
    "      <material name=\"red ish\">"
    "        <color rgba=\"1 0.0001 0.0 1\" />"
    "      </material>"
    "    </visual>"
    "  </link>"
    "</robot>";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(joint_str);

  EXPECT_EQ(2u, urdf->links_.size());
  EXPECT_EQ(2u, urdf->joints_.size());

  EXPECT_EQ(urdf::Geometry::SPHERE, urdf->links_["l1"]->visual->geometry->type);
  std::shared_ptr<urdf::Sphere> s = std::dynamic_pointer_cast<urdf::Sphere>(urdf->links_["l1"]->visual->geometry);
  EXPECT_EQ(1.349, s->radius);
  EXPECT_FLOAT_EQ(1.0, urdf->links_["l1"]->visual->material->color.r);
  EXPECT_FLOAT_EQ(0.65f, static_cast<float>(urdf->links_["l1"]->visual->material->color.g));
  EXPECT_FLOAT_EQ(0.0, urdf->links_["l1"]->visual->material->color.b);
  EXPECT_FLOAT_EQ(0.01f, static_cast<float>(urdf->links_["l1"]->visual->material->color.a));
  EXPECT_EQ("", urdf->links_["l1"]->visual->material->name);
  EXPECT_EQ("", urdf->links_["l1"]->visual->material->texture_filename);

  EXPECT_EQ(urdf::Geometry::CYLINDER, urdf->links_["l2"]->visual->geometry->type);
  std::shared_ptr<urdf::Cylinder> c = std::dynamic_pointer_cast<urdf::Cylinder>(urdf->links_["l2"]->visual->geometry);
  EXPECT_EQ(3.349, c->radius);
  EXPECT_EQ(7.5490, c->length);
  EXPECT_FLOAT_EQ(1.0, urdf->links_["l2"]->visual->material->color.r);
  EXPECT_FLOAT_EQ(0.0001f, static_cast<float>(urdf->links_["l2"]->visual->material->color.g));
  EXPECT_FLOAT_EQ(0.0, urdf->links_["l2"]->visual->material->color.b);
  EXPECT_FLOAT_EQ(1.0, urdf->links_["l2"]->visual->material->color.a);
  EXPECT_EQ("red ish", urdf->links_["l2"]->visual->material->name);
  EXPECT_EQ("", urdf->links_["l2"]->visual->material->texture_filename);

  EXPECT_EQ(8.4396, urdf->links_["l1"]->inertial->mass);
  EXPECT_EQ(0.087, urdf->links_["l1"]->inertial->ixx);
  EXPECT_EQ(0.14, urdf->links_["l1"]->inertial->ixy);
  EXPECT_EQ(0.912, urdf->links_["l1"]->inertial->ixz);
  EXPECT_EQ(0.763, urdf->links_["l1"]->inertial->iyy);
  EXPECT_EQ(0.0012, urdf->links_["l1"]->inertial->iyz);
  EXPECT_EQ(0.908, urdf->links_["l1"]->inertial->izz);
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // use the environment locale so that the unit test can be repeated with various locales easily
  setlocale(LC_ALL, "");

  return RUN_ALL_TESTS();
}
