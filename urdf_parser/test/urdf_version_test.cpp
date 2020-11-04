#include <iostream>
#include <string>

#include <gtest/gtest.h>

#include <urdf_parser/urdf_parser.h>

#define EXPECT_THROW_OF_TYPE(type, statement, msg)                         \
  do {                                                                     \
    try {                                                                  \
      statement;                                                           \
    } catch (const type & err) {                                           \
      if (std::string(err.what()).find(msg) == std::string::npos) {        \
        FAIL() << "Expected error msg containing:" << std::endl            \
               << msg << std::endl                                         \
               << "Saw error msg:" << std::endl                            \
               << err.what() << std::endl;                                 \
      }                                                                    \
    } catch (const std::exception & err) {                                 \
      FAIL() << "Expected " #type << std::endl                             \
             << "Saw exception type: " << typeid(err).name() << std::endl; \
    }                                                                      \
  } while (0)

#define EXPECT_RUNTIME_THROW(st, msg) \
  EXPECT_THROW_OF_TYPE(std::runtime_error, st, msg)


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

  EXPECT_EQ(urdf, nullptr);
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

TEST(URDF_VERSION_CLASS, test_null_ptr)
{
  urdf_export_helpers::URDFVersion vers1(nullptr);

  EXPECT_EQ(vers1.getMajor(), 1u);
  EXPECT_EQ(vers1.getMinor(), 0u);
}

TEST(URDF_VERSION_CLASS, test_correct_string)
{
  urdf_export_helpers::URDFVersion vers2("1.0");

  EXPECT_EQ(vers2.getMajor(), 1u);
  EXPECT_EQ(vers2.getMinor(), 0u);
}

TEST(URDF_VERSION_CLASS, test_too_many_dots)
{
  EXPECT_RUNTIME_THROW(urdf_export_helpers::URDFVersion vers2("1.0.0"),
                       "The version attribute should be in the form 'x.y'");
}

TEST(URDF_VERSION_CLASS, test_not_enough_numbers)
{
  EXPECT_RUNTIME_THROW(urdf_export_helpers::URDFVersion vers2("1."),
                       "The version attribute should be in the form 'x.y'");
}

TEST(URDF_VERSION_CLASS, test_no_major_number)
{
  EXPECT_RUNTIME_THROW(urdf_export_helpers::URDFVersion vers2(".1"),
                       "One of the fields of the version attribute is blank");
}

TEST(URDF_VERSION_CLASS, test_negative_major_number)
{
  EXPECT_RUNTIME_THROW(urdf_export_helpers::URDFVersion vers2("-1.0"),
                       "Version number must be positive");
}

TEST(URDF_VERSION_CLASS, test_negative_minor_number)
{
  EXPECT_RUNTIME_THROW(urdf_export_helpers::URDFVersion vers2("1.-1"),
                       "Version number must be positive");
}

TEST(URDF_VERSION_CLASS, test_no_numbers)
{
  EXPECT_RUNTIME_THROW(urdf_export_helpers::URDFVersion vers2("abc"),
                       "The version attribute should be in the form 'x.y'");
}

TEST(URDF_VERSION_CLASS, test_dots_no_numbers)
{
  EXPECT_RUNTIME_THROW(urdf_export_helpers::URDFVersion vers2("a.c"),
                       "Version attribute is not an integer");
}

TEST(URDF_VERSION_CLASS, test_dots_one_number)
{
  EXPECT_RUNTIME_THROW(urdf_export_helpers::URDFVersion vers2("1.c"),
                       "Version attribute is not an integer");
}

TEST(URDF_VERSION_CLASS, test_trailing_junk)
{
  EXPECT_RUNTIME_THROW(urdf_export_helpers::URDFVersion vers2("1.0~pre6"),
                       "Extra characters after the version number");
}
