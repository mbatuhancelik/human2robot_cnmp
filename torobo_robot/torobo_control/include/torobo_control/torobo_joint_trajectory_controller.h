/**
 * @file  torobo_joint_trajectory_controller.h
 * @brief ToroboJointTrajectoryController class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_JOINT_TRAJECTORY_CONTROLLER_H
#define TOROBO_JOINT_TRAJECTORY_CONTROLLER_H


/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <torobo_msgs/ToroboJointState.h>
#include "torobo_control/torobo_action_controller_base.h"


namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboJointTrajectoryController : public ToroboActionControllerBase
{
public:
    ToroboJointTrajectoryController(ros::NodeHandle& node, std::string name, std::string action_name);
    virtual ~ToroboJointTrajectoryController();

protected:
    void goalCallback();
    void stateCallback(const torobo_msgs::ToroboJointState::ConstPtr& msg);
    void preemptCallback();
    bool validateGoal(control_msgs::FollowJointTrajectoryGoalConstPtr goal);
    bool initJointStateMap(control_msgs::FollowJointTrajectoryGoalConstPtr goal);
    control_msgs::FollowJointTrajectoryResult createResult(int error_code, std::string error_string="");
    void setGoalState(actionlib::SimpleClientGoalState::StateEnum goal_state, int error_code, std::string text="");

    enum ErrorCode
    {
        SUCCESSFUL = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL,
        INVALID_GOAL = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL,
        INVALID_JOINTS = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS,
        OLD_HEADER_TIMESTAMP = control_msgs::FollowJointTrajectoryResult::OLD_HEADER_TIMESTAMP, // not supperted
        PATH_TOLERANCE_VIOLATED = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED, // not supperted
        GOAL_TOLERANCE_VIOLATED = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED,
    } ErrorCode;

private:
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    ros::ServiceClient service_cancel_trajectory_client_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    double GOAL_TOLERANCE_DEFAULT;
    double GOAL_TIME_TOLERANCE_DEFAULT;
    double ADDITIONAL_MONITORING_DURATION;
};

} // namespace torobo

#endif /* TOROBO_JOINT_TRAJECTORY_CONTROLLER_H */

