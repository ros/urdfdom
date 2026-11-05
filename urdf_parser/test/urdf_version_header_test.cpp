#include <string>

#include <gtest/gtest.h>

#include <urdf_parser/version.hpp>
#include <urdf_parser/version.h>  // backwards-compatible alias must also work

TEST(URDFDOM_VERSION_HEADER, version_string_matches_components)
{
  const std::string expected = std::to_string(URDFDOM_VERSION_MAJOR) + "." +
    std::to_string(URDFDOM_VERSION_MINOR) + "." + std::to_string(URDFDOM_VERSION_PATCH);
  EXPECT_EQ(expected, URDFDOM_VERSION_STR);
}

TEST(URDFDOM_VERSION_HEADER, version_matches_build_version)
{
  // TEST_URDFDOM_VERSION is passed from CMake (read from package.xml)
  EXPECT_STREQ(TEST_URDFDOM_VERSION, URDFDOM_VERSION_STR);
}

TEST(URDFDOM_VERSION_HEADER, version_gte)
{
  constexpr int M = URDFDOM_VERSION_MAJOR;
  constexpr int m = URDFDOM_VERSION_MINOR;
  constexpr int p = URDFDOM_VERSION_PATCH;
  static_assert(URDFDOM_VERSION_GTE(M, m, p), "current version must be >= itself");
  EXPECT_TRUE(URDFDOM_VERSION_GTE(0, 0, 0));
  EXPECT_TRUE(URDFDOM_VERSION_GTE(M - 1, m + 1, p + 1));
  EXPECT_TRUE(URDFDOM_VERSION_GTE(M, m - 1, p + 1));
  EXPECT_TRUE(URDFDOM_VERSION_GTE(M, m, p - 1));
  EXPECT_FALSE(URDFDOM_VERSION_GTE(M + 1, 0, 0));
  EXPECT_FALSE(URDFDOM_VERSION_GTE(M, m + 1, 0));
  EXPECT_FALSE(URDFDOM_VERSION_GTE(M, m, p + 1));
}
