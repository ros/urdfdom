#include <gtest/gtest.h>

#include <urdf_model/utils.h>

TEST(URDF_DOUBLE_CONVERT, test_successful_conversion)
{
  std::string easy{"1.0"};
  double conv = urdf::strToDouble(easy.c_str());
  EXPECT_EQ(1.0, conv);

  std::string scientific{"0.00006"};
  double sconv = urdf::strToDouble(scientific.c_str());
  EXPECT_NEAR(0.00006, sconv, 0.00001);
}

TEST(URDF_DOUBLE_CONVERT, test_failed_conversion)
{
  std::string invalid{"foo"};
  EXPECT_THROW({
      try {
        urdf::strToDouble(invalid.c_str());
    } catch(const std::runtime_error& e) {
        EXPECT_STREQ("Failed converting string to double", e.what());
        throw;
    }
    }, std::runtime_error);
}
