/**
 * @file  move_teaching_trajectory_action.h
 * @brief Move teaching trajectory action server class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef MOVE_TEACHING_TRAJECTORY_ACTION_H
#define MOVE_TEACHING_TRAJECTORY_ACTION_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "torobo_msgs/MoveTeachingTrajectoryAction.h"

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
namespace torobo
{

class MoveTeachingTrajectoryActionServer
{
public:
    MoveTeachingTrajectoryActionServer(const ros::NodeHandle &node,
        const std::string& controller_name, const std::vector<std::string>& joint_names,
        const double rate, const double timeout_sec, const bool debug
    );
    virtual ~MoveTeachingTrajectoryActionServer();

    void ActionCallback(const torobo_msgs::MoveTeachingTrajectoryGoalConstPtr &goal);

protected:
    trajectory_msgs::JointTrajectory CallGetTeachingTrajectoryService(const std::string teachingTrajectoryName);
    bool CallFollowJointTrajectoryAction(const trajectory_msgs::JointTrajectory jointTrajectory);

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<torobo_msgs::MoveTeachingTrajectoryAction> as_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;

    const bool debug_;
    const double rate_;
    const double timeout_sec_;
    const std::string controller_name_;
    const std::vector<std::string> joint_names_;
};

}

#endif
