#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <torobo_common/robot_description_parser.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init (argc, argv, "robot_description_parser");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

class RobotDescriptionParserTestFixture : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
    }
    virtual void TearDown()
    {
    }
};

TEST_F(RobotDescriptionParserTestFixture, Parse)
{
    // ros::NodeHandle nh;
    // torobo_common::RobotDescription robot_description;
    // torobo_common::parseRobotDescriptionFromRosParam(robot_description, nh);
}
