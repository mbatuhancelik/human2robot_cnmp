#include <gtest/gtest.h>
#include <ros/package.h>
#include <torobo_common/math_util.h>


using namespace std;

class MathUtilTestFixture : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
    }
    virtual void TearDown()
    {
    }
};

TEST_F(MathUtilTestFixture, Comparison)
{
    double a = 100.0;
    double b = 10.0;
    ASSERT_TRUE(math::nearly_equal(a/b, 10.0));
    ASSERT_TRUE(math::nearly_equal_or_less_than(a/b, 10.0));
    ASSERT_TRUE(math::nearly_equal_or_less_than(a/b, 10.001));
    ASSERT_FALSE(math::nearly_equal_or_less_than(a/b, 9.999));
    ASSERT_TRUE(math::nearly_equal_or_in_range(9.0, 11.0, a/b));
    ASSERT_TRUE(math::nearly_equal_or_in_range(9.0, 10.0, a/b));
    ASSERT_TRUE(math::nearly_equal_or_in_range(10.0, 11.0, a/b));
    ASSERT_FALSE(math::nearly_equal_or_in_range(10.001, 11.0, a/b));
    ASSERT_FALSE(math::nearly_equal_or_in_range(9.0, 9.999, a/b));
}
