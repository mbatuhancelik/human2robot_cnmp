/**
 * @file  move_home_position_action.cpp
 * @brief Move home position action server class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "torobo_motion_manager/move_home_position_action.h"
#include "torobo_common/srdf_parser.h"

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

using namespace std;

/*----------------------------------------------------------------------
 Privat Global Variables
 ----------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
namespace torobo
{

MoveHomePositionActionServer::MoveHomePositionActionServer(const ros::NodeHandle &node,
    const std::string& controller_name, const std::vector<std::string>& joint_names,
    const double rate, const double timeout_sec, const bool debug
)
    : nh_(node), controller_name_(controller_name), joint_names_(joint_names),
        rate_(rate), timeout_sec_(timeout_sec),
        as_(nh_, controller_name + "/move_home_position",
        boost::bind(&MoveHomePositionActionServer::ActionCallback, this, _1), false),
        ac_(controller_name + "/follow_joint_trajectory"),
        debug_(debug)
{
    home_position_ = GetHomePositionFromRosParam();
    as_.start();
}

MoveHomePositionActionServer::~MoveHomePositionActionServer()
{
    as_.shutdown();
}

void MoveHomePositionActionServer::ActionCallback(const torobo_msgs::MoveHomePositionGoalConstPtr &goal)
{
    torobo_msgs::MoveHomePositionFeedback feedback;
    torobo_msgs::MoveHomePositionResult result;
    feedback.actual.positions = vector<double>(joint_names_.size(), 0.0);
    feedback.actual.velocities = vector<double>(joint_names_.size(), 0.0);
    feedback.actual.accelerations = vector<double>(joint_names_.size(), 0.0);
    feedback.actual.effort = vector<double>(joint_names_.size(), 0.0);
    feedback.actual.time_from_start = ros::Duration(0.0);
    result.jointState.name = joint_names_;
    result.jointState.position = vector<double>(joint_names_.size(), 0.0);
    result.jointState.velocity = vector<double>(joint_names_.size(), 0.0);
    result.jointState.effort = vector<double>(joint_names_.size(), 0.0);

    ros::Time lastTime = ros::Time::now();

    ROS_INFO("[%s] Received goal: TransitionTime:%f", controller_name_.c_str(), goal->transitionTime);

    try
    {
        double transitionTime = goal->transitionTime;
        trajectory_msgs::JointTrajectory jointTrajectory = GenerateJointTrajectoryFromJointPosition(home_position_, transitionTime);

        if(debug_)
        {
            ROS_INFO("[%s] Start follow joint trajectory action", controller_name_.c_str());
        }

        if(!CallFollowJointTrajectoryAction(jointTrajectory))
        {
            ROS_ERROR_STREAM("[" << controller_name_ << "] " << __PRETTY_FUNCTION__ << "] FollowJointTrajectory action server does not response.");
            result.errorCode = result.FAILURE;
            as_.setAborted(result);
            return;
        }

        while(true)
        {
            if(as_.isPreemptRequested() || !ros::ok())
            {
                result.errorCode = result.PREEMPTED;
                as_.setPreempted(result);
                ROS_WARN_STREAM("[" << controller_name_ << "] " << __PRETTY_FUNCTION__ << "] setPreempted ");
                return;
            }

            const ros::Duration d(ros::Time::now() - lastTime);
            feedback.actual.time_from_start = d;
            as_.publishFeedback(feedback);

            if(debug_)
            {
                ROS_INFO_STREAM("[" << controller_name_ << "] " << __PRETTY_FUNCTION__ << "Duration: "<< d.toSec());
            }

            if(d.toSec() >= goal->transitionTime)
            {
                result.errorCode = result.SUCCESSFUL;
                as_.setSucceeded(result);
                ROS_INFO_STREAM("[" << controller_name_ << "] " << __PRETTY_FUNCTION__ << "] setSucceeded ");
                return;
            }
            ros::Rate(rate_).sleep();
        }
    }
    catch(const std::exception& e)
    {
        result.errorCode = result.FAILURE;
        ROS_ERROR_STREAM(e.what());
        as_.setAborted(result);
        ROS_WARN_STREAM("[" << controller_name_ << "] " << __PRETTY_FUNCTION__ << "] setAborted ");
    }
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
MoveHomePositionActionServer::JointPosition_t MoveHomePositionActionServer::GetHomePositionFromRosParam()
{
    JointPosition_t home_position;

    torobo_common::SrdfParser::GroupStateMap group_state_map = torobo_common::SrdfParser::getGroupStateFromRosParam(nh_, "robot_description_semantic");
    for(auto itr = group_state_map.begin(); itr != group_state_map.end(); ++itr)
    {
        const string& group_name = itr->first;

        // check controller_name is beginning with group_name or not
        if(controller_name_.size() < group_name.size()
        || !std::equal(std::begin(group_name), std::end(group_name), std::begin(controller_name_))) 
        {
            continue;
        }
        if(itr->second.count("home_position") > 0)
        {
            const torobo_common::SrdfParser::GroupState_t& home_state = itr->second.at("home_position");
            home_position.joint_names = home_state.joint_names;
            home_position.positions = home_state.positions;
            break;
        }
    }

    return home_position;
}

bool MoveHomePositionActionServer::CallFollowJointTrajectoryAction(const trajectory_msgs::JointTrajectory jointTrajectory)
{
    if(!ac_.waitForServer(ros::Duration(timeout_sec_)))
    {
        return false;
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = jointTrajectory;
    ac_.sendGoal(goal);

    return true;
}

trajectory_msgs::JointTrajectory MoveHomePositionActionServer::GenerateJointTrajectoryFromJointPosition(const JointPosition_t jointPosition, const double transitionTime) const
{
    // Make JointTrajectoryPoint
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = jointPosition.positions;
    for(int i = 0; i < jointPosition.joint_names.size(); i++)
    {
        point.velocities.push_back(0.0);
        point.accelerations.push_back(0.0);
        point.effort.push_back(0.0);
    }
    point.time_from_start = ros::Duration(transitionTime);

    // Make JointTrajectory
    trajectory_msgs::JointTrajectory jointTrajectory;
    jointTrajectory.header.stamp = ros::Time::now();
    jointTrajectory.joint_names = jointPosition.joint_names;
    jointTrajectory.points.push_back(point);

    return jointTrajectory;
}

}
