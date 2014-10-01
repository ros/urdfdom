#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <iomanip>
#include <cmath>

// the name of our test module
#define BOOST_TEST_MODULE URDF_UNIT_TEST
// needed for automatic generation of the main()
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>


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
  BOOST_CHECK(quat_are_near(rot0, rot1));
}

BOOST_AUTO_TEST_CASE(test_rotation_get_set_rpy_idempotent)
{
  double x0 = 0.5, y0 = -0.5, z0 = 0.5,  w0 = 0.5;
  check_get_set_rpy_is_idempotent(x0, y0, z0, w0);
  double delta = 2.2e-8;
  check_get_set_rpy_is_idempotent(x0, y0, z0+delta, w0-delta);
}
