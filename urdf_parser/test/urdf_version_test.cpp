#include <gtest/gtest.h>

#include <urdf_parser/urdf_parser.h>

TEST(URDF_VERSION, test_version_wrong_type)
{
  std::string test_str =
    "<robot name=\"test\" version=\"foo\">"
    "</robot>";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(test_str);

  EXPECT_EQ(urdf, nullptr);
}

TEST(URDF_VERSION, test_version_unsupported_version)
{
  std::string test_str =
    "<robot name=\"test\" version=\"2\">"
    "</robot>";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(test_str);

  EXPECT_EQ(urdf, nullptr);
}

TEST(URDF_VERSION, test_version_not_specified)
{
  std::string test_str =
    "<robot name=\"test\">"
    "  <link name=\"l1\"/>"
    "</robot>";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(test_str);

  EXPECT_NE(urdf, nullptr);
}

TEST(URDF_VERSION, test_version_one_int)
{
  std::string test_str =
    "<robot name=\"test\" version=\"1\">"
    "  <link name=\"l1\"/>"
    "</robot>";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(test_str);

  EXPECT_NE(urdf, nullptr);
}

TEST(URDF_VERSION, test_version_one_float)
{
  std::string test_str =
    "<robot name=\"test\" version=\"1.0\">"
    "  <link name=\"l1\"/>"
    "</robot>";

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(test_str);

  EXPECT_NE(urdf, nullptr);
}
