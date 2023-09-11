/**
 * @file  torobo_joint_trajectory_controller.cpp
 * @brief ToroboJointTrajectoryController class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <iostream>
#include <boost/bind.hpp>
#include <torobo_msgs/CancelTrajectory.h>
#include <torobo_msgs/GetJointState.h>
#include <torobo_common/math_util.h>
#include "torobo_control/torobo_joint_trajectory_controller.h"


using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Private Static Const Variables
 ----------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 Public Method Implementations
 ----------------------------------------------------------------------*/
ToroboJointTrajectoryController::ToroboJointTrajectoryController(ros::NodeHandle& node, std::string name, std::string action_name) :
    ToroboActionControllerBase(node, name, action_name),
    as_(*nh_ptr_, name + "/" + action_name, false)
{
    // Get Ros Parameter
    ros::param::param<double>(name + "/" + action_name + "/goal_tolerance_default", GOAL_TOLERANCE_DEFAULT, 0.01745); // 1.0[deg] * MATH_PI / 180.0
    ros::param::param<double>(name + "/" + action_name + "/goal_time_tolerance_default", GOAL_TIME_TOLERANCE_DEFAULT, 0.5);
    ros::param::param<double>(name + "/" + action_name + "/additional_monitoring_period", ADDITIONAL_MONITORING_DURATION, 3.0);

    // Set Publisher and Subscriber and Service
    pub_ = nh_ptr_->advertise<trajectory_msgs::JointTrajectory>(name_ + "/command", 1, this);
    sub_ = nh_ptr_->subscribe("joint_state_server/" + name_ + "/torobo_joint_state", 1, &ToroboJointTrajectoryController::stateCallback, this);

    // Cancel Trajectory Client Service
    service_cancel_trajectory_client_ = nh_ptr_->serviceClient<torobo_msgs::CancelTrajectory>(name_ + "/cancel_trajectory");

    // Register ActionServer's Callbacks
    as_.registerGoalCallback(boost::bind(&ToroboJointTrajectoryController::goalCallback, this));
    as_.registerPreemptCallback(boost::bind(&ToroboJointTrajectoryController::preemptCallback, this));
    as_.start(); // action server's status is "Pending" first.
}

ToroboJointTrajectoryController::~ToroboJointTrajectoryController()
{
    as_.shutdown();
    pub_.shutdown();
    sub_.shutdown();
    service_cancel_trajectory_client_.shutdown();
    async_spinner_ptr_->stop();  // wait for finishing async spinner's thread
}

/*----------------------------------------------------------------------
 Protected Method Implementations
 ----------------------------------------------------------------------*/

void ToroboJointTrajectoryController::stateCallback(const torobo_msgs::ToroboJointState::ConstPtr& msg)
{
    // Check if the action is active
    //   'isActive()' returns true when current goal is valid
    //    and action server's status is "Active" or "Preempting".
    if(!as_.isActive())
    {
        return;
    }

    if(!updateJointstateMap(msg))
    {
        return;
    }

    // Check if the goal_time with tolerance is passed over
    if (!checkTimeNowIsInMonitoringLimit())
    {
        // send cancel_trajectory to ToroboDriver
        callCancelTrajectoryService(service_cancel_trajectory_client_);
        setGoalState(actionlib::SimpleClientGoalState::StateEnum::ABORTED, GOAL_TOLERANCE_VIOLATED, "time is over monitoring limit");
        return;
    }

    // Check if the traj status of all the joints have started trajectory and been running.
    if(!trj_status_running_)
    {
        if(!checkForTrajectoryRunning())
        {
            return;
        }
        trj_status_running_ = true;
        ROS_INFO("[%s] %s: Trajectory movement has been started.", name_.c_str(), action_name_.c_str());
    }

    // Check if the traj status of all the joints have completed trajectory.
    if(!checkForTrajectoryComplete())
    {
        return;
    }
    ROS_INFO("[%s] %s: Trajectory movement has been finished.", name_.c_str(), action_name_.c_str());

    // Set Succeeded and return result.
    if (!checkGoalIsInTolerance())
    {
        setGoalState(actionlib::SimpleClientGoalState::StateEnum::ABORTED, GOAL_TOLERANCE_VIOLATED, "goal tolerance violated");
    }
    else if(!checkGoalTimeIsInTolerance()) // check goal time tolerance
    {
        setGoalState(actionlib::SimpleClientGoalState::StateEnum::ABORTED, GOAL_TOLERANCE_VIOLATED, "goal time tolerance violated");
    }
    else
    {
        setGoalState(actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED, SUCCESSFUL, "");
    }
}

void ToroboJointTrajectoryController::goalCallback()
{
    // This goalCB is called by action server when a user send a goal.
    // If the user send a new goal when an old goal is active yet,
    // First, preemptCB is called, and then goalCB is called by action client.
    // This ROS's specification is written in http://wiki.ros.org/joint_trajectory_controller
    ROS_INFO("[%s] %s: Goal Received", name_.c_str(), action_name_.c_str());

    // Check if the action is active
    //   'isActive()' returns true when current goal is valid
    //    and action server's status is "Active" or "Preempting".
    if(as_.isActive())
    {
        // this block is never reached because PreemptCB is called before GoalCB when double goal is thrown.
        ROS_WARN("[%s] %s: New goal has been ignored because old goal is active.", name_.c_str(), action_name_.c_str());
        return;
    }

    // Accept new goal
    control_msgs::FollowJointTrajectoryGoalConstPtr goal = as_.acceptNewGoal(); // action server's status is changed into "Active".

    // Init traj status flag
    trj_status_running_ = false;

    // Init time variables
    initTimeVariables(
        goal->trajectory.points[goal->trajectory.points.size()-1].time_from_start.toSec(),
        math::nearly_equal(goal->goal_time_tolerance.toSec(), 0.0) ? GOAL_TIME_TOLERANCE_DEFAULT : goal->goal_time_tolerance.toSec(),
        ADDITIONAL_MONITORING_DURATION
    );

    // Init jointstate_map_
    if(!initJointStateMap(goal))
    {
        return;
    }

    // Ignore Path tolerance in FollowJointTrajectoryAction according to torobo robot_controller's specification.

    // Start Action
    pub_.publish(goal->trajectory);
}

void ToroboJointTrajectoryController::preemptCallback()
{
    // preemptCallback is called when a user publishes cancel
    // or when a usser publish multiple goals.
    ROS_INFO("[%s] %s: Preempting", name_.c_str(), action_name_.c_str());

    // send cancel_trajectory to ToroboDriver
    callCancelTrajectoryService(service_cancel_trajectory_client_);
    setGoalState(actionlib::SimpleClientGoalState::StateEnum::PREEMPTED, SUCCESSFUL, "");
}

void ToroboJointTrajectoryController::setGoalState(actionlib::SimpleClientGoalState::StateEnum goal_state, int error_code, std::string text)
{
    std::string goal_status_prefix, goal_status_info;
    goal_status_prefix += "[" + name_ + "] " + action_name_ + ": ";
    goal_status_info += "    [elapsed_time:" + std::to_string((ros::Time::now() - start_time_).toSec());
    goal_status_info += ", position:[" + getJointStateString() + "]]";

    std::string all_text;
    switch(goal_state)
    {
        case actionlib::SimpleClientGoalState::StateEnum::PREEMPTED:
            all_text = goal_status_prefix + "Preempted. " + text + goal_status_info;
            ROS_WARN_STREAM(all_text);
            as_.setPreempted(createResult(error_code, all_text), all_text);
            break;
        case actionlib::SimpleClientGoalState::StateEnum::ABORTED:
            all_text = goal_status_prefix + "Aborted. " + text + goal_status_info;
            ROS_ERROR_STREAM(all_text);
            as_.setAborted(createResult(error_code, all_text), all_text);
            break;
        case actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED:
            all_text = goal_status_prefix + "Succeeded. " + text + goal_status_info;
            ROS_INFO_STREAM(all_text);
            as_.setSucceeded(createResult(error_code, all_text), all_text);
            break;
        default:
            ROS_ERROR_STREAM("invalid goal status");
            as_.setAborted(createResult(error_code, all_text), "invalid goal_status");
            break;
    }
}

bool ToroboJointTrajectoryController::validateGoal(control_msgs::FollowJointTrajectoryGoalConstPtr goal)
{
    for(int i=0; i<goal->trajectory.joint_names.size(); ++i)
    {
        std::string joint_name = goal->trajectory.joint_names[i];
        std::shared_ptr<Joint> joint = getJoint(joint_name);
        // check whether joint name exists in this controller
        if(!joint)
        {
            setGoalState(actionlib::SimpleClientGoalState::StateEnum::ABORTED, INVALID_JOINTS, "joint name [" + joint_name + "] does not exist.");
            return false;
        }
        // check whether trajectory waypoints are between limit_lower and limit_upper
        for(int j=0; j<goal->trajectory.points.size(); ++j)
        {
            double position = goal->trajectory.points[j].positions[i];
            if(!math::nearly_equal_or_in_range(joint->limit_lower, joint->limit_upper, position))
            {
                setGoalState(actionlib::SimpleClientGoalState::StateEnum::ABORTED, INVALID_GOAL,
                    "joint [" + joint_name + "]'s target position [" + std::to_string(position)
                    + "] is not in bound[" + std::to_string(joint->limit_lower) + ":" + std::to_string(joint->limit_upper) + "].");
                return false;
            }
        }
    }
    return true;
}

bool ToroboJointTrajectoryController::initJointStateMap(control_msgs::FollowJointTrajectoryGoalConstPtr goal)
{
    jointstate_map_.clear();

    // Validate goal's joint name and target position
    if(!validateGoal(goal))
    {
        return false;
    }

    for(int i=0; i<goal->trajectory.joint_names.size(); ++i)
    {
        std::string joint_name = goal->trajectory.joint_names[i];
        State state;

        // set goal
        state.goal = goal->trajectory.points[goal->trajectory.points.size()-1].positions[i];

        // set goal_tolerance
        state.goal_tolerance = 0.0;
        for(int j=0; j<goal->goal_tolerance.size(); ++j)
        {
            if(goal->goal_tolerance[j].name == joint_name)
            {
               state.goal_tolerance = goal->goal_tolerance[j].position;
            }
        }
        if (math::nearly_equal(state.goal_tolerance, 0.0))
        {
            // if goal_tolerance is not specified or equals to zero, set default value.
            state.goal_tolerance = GOAL_TOLERANCE_DEFAULT;
        }

        // register to jointstate_map_
        jointstate_map_[joint_name] = state;
    }

    return true;
}

control_msgs::FollowJointTrajectoryResult ToroboJointTrajectoryController::createResult(int error_code, std::string error_string)
{
    control_msgs::FollowJointTrajectoryResult res;
    res.error_code = error_code;
    res.error_string = error_string;
    return res;
}

} // namespace torobo
