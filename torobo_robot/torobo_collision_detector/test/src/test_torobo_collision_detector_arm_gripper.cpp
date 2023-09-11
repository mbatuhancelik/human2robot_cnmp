#include <ros/ros.h>
#include <gtest/gtest.h>
#include <angles/angles.h>
#include <sensor_msgs/JointState.h>
#include "torobo_msgs/CheckCollision.h"
#include "torobo_collision_detector/ToroboCollisionDetectorTest.h"

using namespace std;
using namespace torobo;

int	main(int argc, char **argv)
{
    ros::init(argc, argv, "test_torobo_collision_detetctor_node");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

TEST_F(ToroboCollisionDetectorTest, Arm_Gripper_CheckHomePositionIsNotSelfColliding)
{
    torobo_msgs::CheckCollision::Request req;
    torobo_msgs::CheckCollision::Response res;

    req.jointState.name = joint_names_;
    req.jointState.position = vector<double>(joint_names_.size(), 0.0);
    check_collision_service_client_.call(req, res);

    EXPECT_FALSE(res.isColliding);
}

TEST_F(ToroboCollisionDetectorTest, Arm_Gripper_CheckIsNotSelfColliding)
{
    torobo_msgs::CheckCollision::Request req;
    torobo_msgs::CheckCollision::Response res;

    req.jointState.name = joint_names_;
    req.jointState.position = vector<double>(joint_names_.size(), 0.0);
    req.jointState.position[1] = angles::from_degrees(90.0);
    req.jointState.position[3] = angles::from_degrees(90.0);
    check_collision_service_client_.call(req, res);

    EXPECT_FALSE(res.isColliding);
}

TEST_F(ToroboCollisionDetectorTest, Arm_Gripper_CheckIsSelfColliding)
{
    torobo_msgs::CheckCollision::Request req;
    torobo_msgs::CheckCollision::Response res;

    req.jointState.name = joint_names_;
    req.jointState.position = vector<double>(joint_names_.size(), 0.0);
    req.jointState.position[1] = angles::from_degrees(120.0);
    req.jointState.position[3] = angles::from_degrees(120.0);
    check_collision_service_client_.call(req, res);

    EXPECT_TRUE(res.isColliding);
}