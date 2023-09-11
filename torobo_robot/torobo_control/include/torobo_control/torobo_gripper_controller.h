/**
 * @file  torobo_gripper_controller.h
 * @brief ToroboGripperController class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_GRIPPER_CONTROLLER_H
#define TOROBO_GRIPPER_CONTROLLER_H


/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/GripperCommand.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <torobo_msgs/ToroboJointState.h>
#include "torobo_control/torobo_action_controller_base.h"


namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboGripperController : public ToroboActionControllerBase
{
public:
    ToroboGripperController(ros::NodeHandle& node, std::string name, std::string action_name);
    virtual ~ToroboGripperController();

protected:
    void goalCallback();
    void stateCallback(const torobo_msgs::ToroboJointState::ConstPtr& msg);
    void preemptCallback();
    bool validateGoal(control_msgs::GripperCommandGoalConstPtr goal);
    bool initJointStateMap(control_msgs::GripperCommandGoalConstPtr goal);
    control_msgs::GripperCommandResult createResult(bool stalled, bool reached_goal);
    void setGoalState(actionlib::SimpleClientGoalState::StateEnum goal_state, bool stalled, bool reached_goal, std::string text="");

private:
    actionlib::SimpleActionServer<control_msgs::GripperCommandAction> as_;
    ros::ServiceClient service_cancel_trajectory_client_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    double GOAL_TOLERANCE_DEFAULT = 0.001;
    double GOAL_TIME_TOLERANCE_DEFAULT;
    double ADDITIONAL_MONITORING_DURATION;
};

} // namespace torobo

#endif /* TOROBO_GRIPPER_CONTROLLER_H */
