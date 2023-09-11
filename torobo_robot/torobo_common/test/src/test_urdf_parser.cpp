#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <torobo_common/urdf_parser.h>

using namespace std;

class UrdfParserTestFixture : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
    }
    virtual void TearDown()
    {
    }
};

TEST_F(UrdfParserTestFixture, ParseToroboArmUrdf)
{
    ros::NodeHandle nh;

    // URDF model
    urdf::ModelSharedPtr urdf = torobo_common::UrdfParser::getUrdf(nh, "robot_description");
    ASSERT_TRUE(urdf != NULL);

    urdf::JointConstSharedPtr urdf_joint = torobo_common::UrdfParser::getUrdfJoint(*urdf, "arm/joint_1");
    ASSERT_TRUE(urdf_joint != NULL);

    ASSERT_TRUE(urdf_joint->name == "arm/joint_1");
    ASSERT_TRUE(urdf_joint->limits->lower < 0.0);
    ASSERT_TRUE(urdf_joint->limits->upper > 0.0);
    ASSERT_TRUE(urdf_joint->limits->effort > 0.0);
    ASSERT_TRUE(urdf_joint->limits->velocity > 0.0);
}
