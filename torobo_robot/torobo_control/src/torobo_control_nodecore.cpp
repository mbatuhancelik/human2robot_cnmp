/**
 * @file  torobo_control_nodecore.cpp
 * @brief ToroboControlNodeCore class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <string>
#include <map>
#include "torobo_control/torobo_control_nodecore.h"
#include "torobo_control/torobo_controller_spawner.h"
#include "torobo_control/torobo_joint_state_server.h"
#include "torobo_control/torobo_joint_state_controller.h"
#include "torobo_control/torobo_joint_trajectory_controller.h"
#include "torobo_control/torobo_gripper_controller.h"


using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Public Method Implementations
 ----------------------------------------------------------------------*/
ToroboControlNodeCore::ToroboControlNodeCore(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh)
{
    controller_spawner_.reset(new ToroboControllerSpawner(nh_));
    joint_state_server_.reset(new ToroboJointStateServer(nh_));
    ROS_INFO("ToroboControlNodeCore Ready.");
}

ToroboControlNodeCore::~ToroboControlNodeCore()
{
}

void ToroboControlNodeCore::run()
{
    // launch controller_spawner
    bool spawn = true;
    private_nh_.param<bool>("spawn", spawn, true);
    if (spawn)
    {
        controller_spawner_->run();
    }

    // launch joint_state_server
    double rate = 20.0;
    private_nh_.param<double>("server_rate", rate, 20.0);
    joint_state_server_->setPublishRate(rate);
    joint_state_server_->registerSourceTopics();
    joint_state_server_->start();
}

} // namespace torobo
