/**
 * @file  move_teaching_point_action.cpp
 * @brief Move teaching point action server class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "torobo_motion_manager/move_teaching_point_action.h"
#include "torobo_msgs/GetTeachingPoint.h"
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

MoveTeachingPointActionServer::MoveTeachingPointActionServer(const ros::NodeHandle &node,
    const std::string& controller_name, const std::vector<std::string>& joint_names,
    const double rate, const double timeout_sec, const bool debug
)
    : nh_(node), controller_name_(controller_name), joint_names_(joint_names),
        rate_(rate), timeout_sec_(timeout_sec),
        as_(nh_, controller_name + "/move_teaching_point",
        boost::bind(&MoveTeachingPointActionServer::ActionCallback, this, _1), false),
        ac_(controller_name + "/follow_joint_trajectory"),
        debug_(debug)
{
    as_.start();
}

MoveTeachingPointActionServer::~MoveTeachingPointActionServer()
{
    as_.shutdown();
}

void MoveTeachingPointActionServer::ActionCallback(const torobo_msgs::MoveTeachingPointGoalConstPtr &goal)
{
    torobo_msgs::MoveTeachingPointFeedback feedback;
    torobo_msgs::MoveTeachingPointResult result;
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

    ROS_INFO("[%s] Received goal: TP Name:%s, TransitionTime:%f", controller_name_.c_str(), goal->teachingPointName.c_str(), goal->transitionTime);

    try
    {
        vector<double> positions = CallGetTeachingPointService(goal->teachingPointName);
        double transitionTime = goal->transitionTime;
        trajectory_msgs::JointTrajectory jointTrajectory = GenerateJointTrajectoryFromPositions(positions, transitionTime);

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
std::vector<double> MoveTeachingPointActionServer::CallGetTeachingPointService(const std::string teachingPointName)
{
    ros::ServiceClient client = nh_.serviceClient<torobo_msgs::GetTeachingPoint>(controller_name_ + "/get_teaching_point");
    torobo_msgs::GetTeachingPoint srv;
    srv.request.teachingPointName = teachingPointName;
    bool ret = client.call(srv);
    if(!ret || !(srv.response.success))
    {
        throw std::runtime_error("Not found teaching point");
    }
    std::string str;
    for(int i = 0; i < srv.response.point.positions.size(); i++)
    {
        str += (to_string(srv.response.point.positions[i]) + ", ");
    }
    if(debug_)
    {
        ROS_INFO_STREAM("[" << controller_name_ << "] " << __PRETTY_FUNCTION__ << "TP Pose: [" << str << "]"); 
    }
    return srv.response.point.positions;
}

bool MoveTeachingPointActionServer::CallFollowJointTrajectoryAction(const trajectory_msgs::JointTrajectory jointTrajectory)
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

trajectory_msgs::JointTrajectory MoveTeachingPointActionServer::GenerateJointTrajectoryFromPositions(const std::vector<double> positions, const double transitionTime) const
{
    // Make JointTrajectoryPoint from teaching point
    trajectory_msgs::JointTrajectoryPoint point;
    for(int i = 0; i < joint_names_.size(); i++)
    {
        point.positions.push_back(positions[i]);
        point.velocities.push_back(0.0);
        point.accelerations.push_back(0.0);
        point.effort.push_back(0.0);
    }
    point.time_from_start = ros::Duration(transitionTime);

    // Make JointTrajectory
    trajectory_msgs::JointTrajectory jointTrajectory;
    jointTrajectory.header.stamp = ros::Time::now();
    jointTrajectory.joint_names = joint_names_;
    jointTrajectory.points.push_back(point);

    return jointTrajectory;
}

}
