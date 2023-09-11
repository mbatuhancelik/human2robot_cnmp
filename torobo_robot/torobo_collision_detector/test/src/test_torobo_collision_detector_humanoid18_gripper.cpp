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

TEST_F(ToroboCollisionDetectorTest, Humanoid18_Gripper_CheckIsNotSelfColliding_1) // Zero Position
{
    torobo_msgs::CheckCollision::Request req;
    torobo_msgs::CheckCollision::Response res;

    req.jointState.name = joint_names_;
    req.jointState.position = vector<double>(joint_names_.size(), 0.0);
    check_collision_service_client_.call(req, res);

    EXPECT_FALSE(res.isColliding);
}

TEST_F(ToroboCollisionDetectorTest, Humanoid18_Gripper_CheckIsNotSelfColliding_2) // Home Position
{
    torobo_msgs::CheckCollision::Request req;
    torobo_msgs::CheckCollision::Response res;

    req.jointState.name = joint_names_;
    req.jointState.position = vector<double>(joint_names_.size(), 0.0);
    req.jointState.position[1] = angles::from_degrees(90.0);
    check_collision_service_client_.call(req, res);

    EXPECT_FALSE(res.isColliding);
}

TEST_F(ToroboCollisionDetectorTest, Humanoid18_Gripper_CheckIsNotSelfColliding_3)
{
    torobo_msgs::CheckCollision::Request req;
    torobo_msgs::CheckCollision::Response res;

    req.jointState.name = joint_names_;
    req.jointState.position = vector<double>(joint_names_.size(), 0.0);
    req.jointState.position[1] = angles::from_degrees(90.0);
    req.jointState.position[3] = angles::from_degrees(-20.0);
    check_collision_service_client_.call(req, res);

    EXPECT_FALSE(res.isColliding);
}

TEST_F(ToroboCollisionDetectorTest, Humanoid18_Gripper_CheckIsSelfColliding_1)
{
    torobo_msgs::CheckCollision::Request req;
    torobo_msgs::CheckCollision::Response res;

    req.jointState.name = joint_names_;
    req.jointState.position = vector<double>(joint_names_.size(), 0.0);
    req.jointState.position[1] = angles::from_degrees(100.0);
    req.jointState.position[3] = angles::from_degrees(0.0);
    check_collision_service_client_.call(req, res);

    EXPECT_TRUE(res.isColliding);
}

TEST_F(ToroboCollisionDetectorTest, Humanoid18_Gripper_CheckIsNotSelfColliding_4)
{
    torobo_msgs::CheckCollision::Request req;
    torobo_msgs::CheckCollision::Response res;

    req.jointState.name = joint_names_;
    req.jointState.position = vector<double>(joint_names_.size(), 0.0);
    req.jointState.position[0] = angles::from_degrees(180.0);
    req.jointState.position[1] = angles::from_degrees(90.0);
    req.jointState.position[3] = angles::from_degrees(60.0);
    check_collision_service_client_.call(req, res);

    EXPECT_FALSE(res.isColliding);
}

TEST_F(ToroboCollisionDetectorTest, Humanoid18_Gripper_CheckIsSelfColliding_2)
{
    torobo_msgs::CheckCollision::Request req;
    torobo_msgs::CheckCollision::Response res;

    req.jointState.name = joint_names_;
    req.jointState.position = vector<double>(joint_names_.size(), 0.0);
    req.jointState.position[0] = angles::from_degrees(180.0);
    req.jointState.position[1] = angles::from_degrees(90.0);
    req.jointState.position[3] = angles::from_degrees(100.0);
    check_collision_service_client_.call(req, res);

    EXPECT_TRUE(res.isColliding);
}

TEST_F(ToroboCollisionDetectorTest, Humanoid18_Gripper_CheckIsNotSelfColliding_5)
{
    torobo_msgs::CheckCollision::Request req;
    torobo_msgs::CheckCollision::Response res;

    req.jointState.name = joint_names_;
    req.jointState.position = vector<double>(joint_names_.size(), 0.0);
    req.jointState.position[0] = angles::from_degrees(90.0);
    req.jointState.position[1] = angles::from_degrees(105.0);  // max of joint_2
    check_collision_service_client_.call(req, res);

    EXPECT_FALSE(res.isColliding);
}

TEST_F(ToroboCollisionDetectorTest, Humanoid18_Gripper_CheckIsNotSelfColliding_6)
{
    torobo_msgs::CheckCollision::Request req;
    torobo_msgs::CheckCollision::Response res;

    req.jointState.name = joint_names_;
    req.jointState.position = vector<double>(joint_names_.size(), 0.0);
    req.jointState.position[3] = angles::from_degrees(115.0);  // max of joint_4
    check_collision_service_client_.call(req, res);

    EXPECT_FALSE(res.isColliding);
}

TEST_F(ToroboCollisionDetectorTest, Humanoid18_Gripper_CheckIsNotSelfColliding_7)
{
    torobo_msgs::CheckCollision::Request req;
    torobo_msgs::CheckCollision::Response res;

    req.jointState.name = joint_names_;
    req.jointState.position = vector<double>(joint_names_.size(), 0.0);
    req.jointState.position[5] = angles::from_degrees(90.0);  // max of joint_6
    check_collision_service_client_.call(req, res);
    EXPECT_TRUE(res.isColliding);
}
