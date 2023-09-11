/**
 * @file  move_teaching_trajectory_action.cpp
 * @brief Move teaching trajectory action server class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "torobo_motion_manager/move_teaching_trajectory_action.h"
#include "torobo_msgs/GetTeachingTrajectory.h"
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

MoveTeachingTrajectoryActionServer::MoveTeachingTrajectoryActionServer(const ros::NodeHandle &node,
    const std::string& controller_name, const std::vector<std::string>& joint_names,
    const double rate, const double timeout_sec, const bool debug
)
    : nh_(node), controller_name_(controller_name), joint_names_(joint_names),
        rate_(rate), timeout_sec_(timeout_sec),
        as_(nh_, controller_name + "/move_teaching_trajectory",
        boost::bind(&MoveTeachingTrajectoryActionServer::ActionCallback, this, _1), false),
        ac_(controller_name + "/follow_joint_trajectory"),
        debug_(debug)
{
    as_.start();
}

MoveTeachingTrajectoryActionServer::~MoveTeachingTrajectoryActionServer()
{
    as_.shutdown();
}

void MoveTeachingTrajectoryActionServer::ActionCallback(const torobo_msgs::MoveTeachingTrajectoryGoalConstPtr &goal)
{
    torobo_msgs::MoveTeachingTrajectoryFeedback feedback;
    torobo_msgs::MoveTeachingTrajectoryResult result;
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

    ROS_INFO("[%s] Received goal: Traj Name:%s", controller_name_.c_str(), goal->teachingTrajectoryName.c_str());

    try
    {
        trajectory_msgs::JointTrajectory jointTrajectory = CallGetTeachingTrajectoryService(goal->teachingTrajectoryName);
        double endTime = jointTrajectory.points.back().time_from_start.toSec();
        jointTrajectory.joint_names = joint_names_;
        ROS_INFO("[%s] Traj length:%d, Traj end time:%lf", controller_name_.c_str(), (int)jointTrajectory.points.size(), endTime);

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

            if(d.toSec() >= endTime)
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
trajectory_msgs::JointTrajectory MoveTeachingTrajectoryActionServer::CallGetTeachingTrajectoryService(const std::string teachingTrajectoryName)
{
    ros::ServiceClient client = nh_.serviceClient<torobo_msgs::GetTeachingTrajectory>(controller_name_ + "/get_teaching_trajectory");
    torobo_msgs::GetTeachingTrajectory srv;
    srv.request.teachingTrajectoryName = teachingTrajectoryName;
    bool ret = client.call(srv);
    if(!ret || !(srv.response.success))
    {
        throw std::runtime_error("Not found teaching trajectory");
    }
    return srv.response.trajectory;
}

bool MoveTeachingTrajectoryActionServer::CallFollowJointTrajectoryAction(const trajectory_msgs::JointTrajectory jointTrajectory)
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

}
