/**
 * @file  move_home_position_action.h
 * @brief Move home position action server class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef MOVE_HOME_POSITION_ACTION_H
#define MOVE_HOME_POSITION_ACTION_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "torobo_msgs/MoveHomePositionAction.h"

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
namespace torobo
{

class MoveHomePositionActionServer
{
public:
    typedef struct
    {
        std::vector<std::string> joint_names;
        std::vector<double> positions;
    }JointPosition_t;

    MoveHomePositionActionServer(const ros::NodeHandle &node,
        const std::string& controller_name, const std::vector<std::string>& joint_names,
        const double rate, const double timeout_sec, const bool debug
    );
    virtual ~MoveHomePositionActionServer();

    void ActionCallback(const torobo_msgs::MoveHomePositionGoalConstPtr &goal);

protected:
    MoveHomePositionActionServer::JointPosition_t GetHomePositionFromRosParam();
    bool CallFollowJointTrajectoryAction(const trajectory_msgs::JointTrajectory jointTrajectory);
    trajectory_msgs::JointTrajectory GenerateJointTrajectoryFromJointPosition(const JointPosition_t jointPosition, const double transitionTime) const;

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<torobo_msgs::MoveHomePositionAction> as_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;

    const bool debug_;
    const double rate_;
    const double timeout_sec_;
    const std::string controller_name_;
    const std::vector<std::string> joint_names_;
    JointPosition_t home_position_;
};

}

#endif
