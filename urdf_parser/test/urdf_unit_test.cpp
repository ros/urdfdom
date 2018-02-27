#include <gtest/gtest.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include "urdf_model/pose.h"

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
