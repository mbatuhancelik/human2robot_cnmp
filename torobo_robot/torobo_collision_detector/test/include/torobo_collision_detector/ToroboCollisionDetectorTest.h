/**
 * @file  ToroboCollisionDetectorTest.h
 * @brief ToroboCollisionDetectorTest class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_COLLISION_DETECTOR_TEST_H
#define TOROBO_COLLISION_DETECTOR_TEST_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <memory>
#include <sensor_msgs/JointState.h>
#include "torobo_msgs/CheckCollision.h"

using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/

class ToroboCollisionDetectorTest : public ::testing::Test
{
public:
    // member
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    string controller_name_;
    string service_name_for_check_collision_;
    vector<string> joint_names_;
    ros::ServiceClient check_collision_service_client_;

    ToroboCollisionDetectorTest()
    {
        // node handler
        nh_ = ros::NodeHandle();
        private_nh_ = ros::NodeHandle("~");

        // get params
        private_nh_.param<string>("controller_name", controller_name_, "arm_controller");
        private_nh_.param<string>("service_name_for_check_collision", service_name_for_check_collision_, "check_collision");

        // get joint list
        joint_names_ = getJointNames(nh_, controller_name_);

        // service client
        check_collision_service_client_ = nh_.serviceClient<torobo_msgs::CheckCollision>(service_name_for_check_collision_);

        // check service exists
        EXPECT_TRUE(check_collision_service_client_.waitForExistence(ros::Duration(10.0)));
    }

    ~ToroboCollisionDetectorTest()
    {
    }

    virtual void SetUp()
    {
    }
    virtual void TearDown()
    {
    }

private:
    vector<string> getJointNames(ros::NodeHandle nh, const string& controller_name) const
    {
        vector<string> joint_names;
        nh.param<vector<string>>(controller_name + "/joints", joint_names, vector<string>());
        return joint_names;
    }
};

}

#endif
