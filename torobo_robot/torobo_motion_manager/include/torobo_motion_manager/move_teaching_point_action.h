/**
 * @file  move_teaching_point_action.h
 * @brief Move teaching point action server class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef MOVE_TEACHING_POINT_ACTION_H
#define MOVE_TEACHING_POINT_ACTION_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "torobo_msgs/MoveTeachingPointAction.h"

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
namespace torobo
{

class MoveTeachingPointActionServer
{
public:
    MoveTeachingPointActionServer(const ros::NodeHandle &node,
        const std::string& controller_name, const std::vector<std::string>& joint_names,
        const double rate, const double timeout_sec, const bool debug
    );
    virtual ~MoveTeachingPointActionServer();

    void ActionCallback(const torobo_msgs::MoveTeachingPointGoalConstPtr &goal);

protected:
    std::vector<double> CallGetTeachingPointService(const std::string teachingPointName);
    bool CallFollowJointTrajectoryAction(const trajectory_msgs::JointTrajectory jointTrajectory);
    trajectory_msgs::JointTrajectory GenerateJointTrajectoryFromPositions(const std::vector<double> positions, const double transitionTime) const;

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<torobo_msgs::MoveTeachingPointAction> as_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;


    const bool debug_;
    const double rate_;
    const double timeout_sec_;
    const std::string controller_name_;
    const std::vector<std::string> joint_names_;
};

}

#endif
