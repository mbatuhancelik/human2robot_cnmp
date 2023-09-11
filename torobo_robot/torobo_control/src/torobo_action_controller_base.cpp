/**
 * @file  torobo_action_controller_base.cpp
 * @brief ToroboActionControllerBase class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <torobo_msgs/ToroboJointState.h>
#include <torobo_msgs/CancelTrajectory.h>
#include <torobo_common/math_util.h>
#include "torobo_control/torobo_action_controller_base.h"


using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Private Static Const Variables
 ----------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 Public Method Implementations
 ----------------------------------------------------------------------*/
ToroboActionControllerBase::ToroboActionControllerBase(ros::NodeHandle& node, std::string name, std::string action_name)
    : name_(name), action_name_(action_name)
{
    nh_ptr_.reset(new ros::NodeHandle(node));
    nh_ptr_->setCallbackQueue(&callback_queue_);
    callback_queue_.callAvailable(ros::WallDuration());
    async_spinner_ptr_.reset(new ros::AsyncSpinner(1, &callback_queue_));
    async_spinner_ptr_->start();
}

ToroboActionControllerBase::~ToroboActionControllerBase()
{
}

void ToroboActionControllerBase::registerJoint(std::string joint_name, double limit_lower, double limit_upper, double limit_effort, double limit_velocity)
{
    std::shared_ptr<Joint> joint(new Joint(joint_name, limit_lower, limit_upper, limit_effort, limit_velocity));
    joints_.push_back(joint);
}

/*----------------------------------------------------------------------
 Protected Method Implementations
 ----------------------------------------------------------------------*/

bool ToroboActionControllerBase::hasJoint(std::string joint_name)
{
    for (const auto joint : joints_)
    {
        if(joint->name == joint_name)
        {
            return true;
        }
    }
    return false;
}

std::shared_ptr<ToroboActionControllerBase::Joint> ToroboActionControllerBase::getJoint(std::string joint_name)
{
    for (const auto joint : joints_)
    {
        if(joint->name == joint_name)
        {
            return joint;
        }
    }
    return nullptr;
}

std::shared_ptr<ToroboActionControllerBase::Joint> ToroboActionControllerBase::getJoint(int index)
{
    if (index < 0 || index >= joints_.size())
    {
        return nullptr;
    }
    return joints_[index];
}

bool ToroboActionControllerBase::updateJointstateMap(const torobo_msgs::ToroboJointState::ConstPtr& msg)
{
    for(auto itr = jointstate_map_.begin(); itr != jointstate_map_.end(); ++itr)
    {
        std::string joint_name = itr->first;
        size_t index = std::distance(msg->name.begin(), std::find(msg->name.begin(), msg->name.end(), joint_name));
        if (index == msg->name.size())
        {
            ROS_ERROR("[%s] %s: joint_name [%s] does not exist in torobo_joint_state.", name_.c_str(), action_name_.c_str(), joint_name.c_str());
            return false;
        }
        itr->second.position = msg->position[index];
        itr->second.velocity = msg->velocity[index];
        itr->second.effort = msg->effort[index];
        itr->second.trjStatus = msg->trjStatus[index];
    }
    return true;
}

bool ToroboActionControllerBase::checkTimeNowIsInMonitoringLimit()
{
    if(ros::Time::now() > goal_time_ + goal_time_tolerance_ + additional_monitoring_duration_)
    {
        return false;
    }
    return true;
}

bool ToroboActionControllerBase::checkForTrajectoryRunning(bool immediate_complete_is_possible)
{
    for(auto itr = jointstate_map_.begin(); itr != jointstate_map_.end(); ++itr)
    {
        if (itr->second.trjStatus == TRAJ_STATUS_RUNNING) // RUNNING
        {
            continue; // OK
        }

        if(immediate_complete_is_possible) // move by pos command not by tpts command
        {
            if(math::nearly_equal_or_in_range(itr->second.goal - itr->second.goal_tolerance, itr->second.goal + itr->second.goal_tolerance, itr->second.position))
            {
                continue; // OK
            }
        }
        return false; // not satisfy condition
    }
    return true; // satisfy condition
}

bool ToroboActionControllerBase::checkForTrajectoryComplete()
{
    // Check if the traj status of all the joints have completed trajectory.
    for(auto itr = jointstate_map_.begin(); itr != jointstate_map_.end(); ++itr)
    {
        if (itr->second.trjStatus == TRAJ_STATUS_COMPLETE) // RUNNING
        {
            continue; // OK
        }
        return false; // not satisfy condition
    }
    return true; // satisfy condition
}

bool ToroboActionControllerBase::callCancelTrajectoryService(ros::ServiceClient& service_client)
{
    // send cancel_trajectory to ToroboDriver
    torobo_msgs::CancelTrajectory srv;
    for (auto itr = jointstate_map_.begin(); itr != jointstate_map_.end(); ++itr)
    {
        std::string joint_name = itr->first;
        srv.request.joint_names.push_back(joint_name);
    }
    ROS_INFO("[%s] %s: calling cancel service", name_.c_str(), action_name_.c_str());
    if(service_client.call(srv))
    {
        ROS_INFO("[%s] %s: cancel service is called", name_.c_str(), action_name_.c_str());
        waitForTrajectoryCancelComplete(5.0);
    }
}

bool ToroboActionControllerBase::waitForTrajectoryCancelComplete(double timeout)
{
    ros::Time limit_time = ros::Time::now() + ros::Duration(timeout);
    while(ros::Time::now() < limit_time)
    {
        torobo_msgs::ToroboJointState::ConstPtr msg = ros::topic::waitForMessage<torobo_msgs::ToroboJointState>("joint_state_server/" + name_ + "/torobo_joint_state", *nh_ptr_);
        if(!updateJointstateMap(msg))
        {
            return false;
        }
        int cancel_count = 0;
        for (auto itr = jointstate_map_.begin(); itr != jointstate_map_.end(); ++itr)
        {
            if (itr->second.trjStatus == TRAJ_STATUS_CANCEL_COMPLETE) // Check COMPLETE
            {
                cancel_count += 1;
            }
        }
        if (cancel_count == jointstate_map_.size())
        {
            return true;
        }
        ros::Rate(20).sleep();
    }

    return false;
}

bool ToroboActionControllerBase::checkGoalIsInTolerance()
{
    for (auto itr = jointstate_map_.begin(); itr != jointstate_map_.end(); ++itr)
    {
        std::string joint_name = itr->first;
        double goal = itr->second.goal;
        double tolerance = itr->second.goal_tolerance;
        double current_position = itr->second.position;
        if(!math::nearly_equal_or_in_range(goal - tolerance, goal + tolerance, current_position))
        {
            ROS_INFO_STREAM("  joint_name[" << joint_name << "]'s check: goal=" << goal << ", tolerance=" << tolerance << ", current=" << current_position);
            return false;
        }
    }
    return true;
}

bool ToroboActionControllerBase::checkGoalTimeIsInTolerance()
{
    if(ros::Time::now() > goal_time_ + goal_time_tolerance_)
    {
        return false;
    }
    return true;
}

std::string ToroboActionControllerBase::getJointStateString()
{
    std::stringstream ss;
    for (auto itr = jointstate_map_.begin(); itr != jointstate_map_.end(); ++itr)
    {
        ss << ", " << itr->first << ": " << itr->second.position;
    }
    return (ss.str().size() < 2 ? "" : ss.str().substr(2));
}

bool ToroboActionControllerBase::initTimeVariables(double time_from_start, double goal_time_tolerance, double additional_duration)
{
    start_time_ = ros::Time::now();
    goal_time_ = start_time_ + ros::Duration(time_from_start);
    goal_time_tolerance_ = ros::Duration(goal_time_tolerance);
    additional_monitoring_duration_ = ros::Duration(additional_duration);
}

} // namespace torobo
