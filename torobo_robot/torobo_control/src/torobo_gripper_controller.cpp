/**
 * @file  torobo_gripper_controller.cpp
 * @brief ToroboGripperController class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <boost/bind.hpp>
#include <torobo_msgs/CancelTrajectory.h>
#include <torobo_msgs/GetJointState.h>
#include <torobo_common/math_util.h>
#include "torobo_control/torobo_gripper_controller.h"


using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Private Static Const Variables
 ----------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 Public Method Implementations
 ----------------------------------------------------------------------*/
ToroboGripperController::ToroboGripperController(ros::NodeHandle& node, std::string name, std::string action_name) :
    ToroboActionControllerBase(node, name, action_name),
    as_(*nh_ptr_, name + "/" + action_name, false)
{
    // Get Ros Parameter
    ros::param::param<double>(name + "/" + action_name + "/goal_tolerance_default", GOAL_TOLERANCE_DEFAULT, 0.001);
    ros::param::param<double>(name + "/" + action_name + "/goal_time_tolerance_default", GOAL_TIME_TOLERANCE_DEFAULT, 8.0);
    ros::param::param<double>(name + "/" + action_name + "/additional_monitoring_period", ADDITIONAL_MONITORING_DURATION, 0.0);

    // Set Publisher and Subscriber
    pub_ = nh_ptr_->advertise<control_msgs::GripperCommand>(name_ + "/command", 1, this);
    sub_ = nh_ptr_->subscribe("joint_state_server/" + name_ + "/torobo_joint_state", 1, &ToroboGripperController::stateCallback, this);

    // Cancel Trajectory Client Service
    service_cancel_trajectory_client_ = nh_ptr_->serviceClient<torobo_msgs::CancelTrajectory>(name_ + "/cancel_trajectory");

    // Register ActionServer's Callbacks
    as_.registerGoalCallback(boost::bind(&ToroboGripperController::goalCallback, this));
    as_.registerPreemptCallback(boost::bind(&ToroboGripperController::preemptCallback, this));
    as_.start();
}

ToroboGripperController::~ToroboGripperController()
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

void ToroboGripperController::stateCallback(const torobo_msgs::ToroboJointState::ConstPtr& msg)
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

    // Check if the goal_time is passed.
    if (!checkTimeNowIsInMonitoringLimit())
    {
        // send cancel_trajectory to ToroboDriver
        callCancelTrajectoryService(service_cancel_trajectory_client_);
        setGoalState(actionlib::SimpleClientGoalState::StateEnum::ABORTED, false, false, " time is over monitoring period.");
        return;
    }

    // Check if the traj status of all the joints have started trajectory and been running.
    if(!trj_status_running_)
    {
        if(!checkForTrajectoryRunning(true))
        {
            return;
        }
        trj_status_running_ = true;
        ROS_INFO("[%s] %s: Gripper movement has been started.", name_.c_str(), action_name_.c_str());
    }

    // Check if the traj status of all the joints have completed trajectory.
    if(!checkForTrajectoryComplete())
    {
        return;
    }
    ROS_INFO("[%s] %s: Gripper movement has been finished.", name_.c_str(), action_name_.c_str());

    // Check "reached_goal" and "stalled" by evaluating position.
    if(!checkGoalIsInTolerance())
    {
        setGoalState(actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED, true, false, "gripper stalled.");
    }
    else
    {
        setGoalState(actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED, false, true, "gripper reached goal.");
    }
}

void ToroboGripperController::goalCallback()
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
    control_msgs::GripperCommandGoalConstPtr goal = as_.acceptNewGoal();  // action server's status is changed into "Active".

    // Init traj status flag
    trj_status_running_ = false;

    // Init time variables
    initTimeVariables(
        0.0,  // transition_time doesn't exist in gripper command.
        GOAL_TIME_TOLERANCE_DEFAULT,
        ADDITIONAL_MONITORING_DURATION
    );

    // Init jointstate_map_
    if(!initJointStateMap(goal))
    {
        return;
    }

    // Start Action
    pub_.publish(goal->command);
}

void ToroboGripperController::preemptCallback()
{
    // preemptCallback is called when a user publishes cancel
    // or when a usser publish multiple goals.
    ROS_INFO("[%s] %s: Preempting", name_.c_str(), action_name_.c_str());

    // send cancel_trajectory to ToroboDriver
    callCancelTrajectoryService(service_cancel_trajectory_client_);
    setGoalState(actionlib::SimpleClientGoalState::StateEnum::PREEMPTED, false, false, "");
}

void ToroboGripperController::setGoalState(actionlib::SimpleClientGoalState::StateEnum goal_state, bool stalled, bool reached_goal, std::string text)
{
    auto res = createResult(stalled, reached_goal);
    std::string goal_status_prefix, goal_status_info;
    goal_status_prefix += "[" + name_ + "] " + action_name_ + ": ";
    goal_status_info += "    [position:" + std::to_string(res.position) + ", effort:" + std::to_string(res.effort);
    goal_status_info += ", stalled:" + std::to_string(res.stalled) + ", reached_goal:" + std::to_string(res.reached_goal);
    goal_status_info += ", elapsed_time:" + std::to_string((ros::Time::now() - start_time_).toSec()) + "]";
    
    std::string all_text;
    switch(goal_state)
    {
        case actionlib::SimpleClientGoalState::StateEnum::PREEMPTED:
            all_text = goal_status_prefix + "Preempted. " + text + goal_status_info;
            ROS_WARN_STREAM(all_text);
            as_.setPreempted(res, all_text);
            break;
        case actionlib::SimpleClientGoalState::StateEnum::ABORTED:
            all_text = goal_status_prefix + "Aborted. " + text + goal_status_info;
            ROS_ERROR_STREAM(all_text);
            as_.setAborted(res, all_text);
            break;
        case actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED:
            all_text = goal_status_prefix + "Succeeded. " + text + goal_status_info;
            ROS_INFO_STREAM(all_text);
            as_.setSucceeded(res, all_text);
            break;
        default:
            ROS_ERROR_STREAM("invalid goal status");
            as_.setAborted(res, "invalid goal_status");
            break;
    }
}

bool ToroboGripperController::validateGoal(control_msgs::GripperCommandGoalConstPtr goal)
{
    std::shared_ptr<Joint> joint = getJoint(0);
    std::string joint_name = joint->name;
    double position = goal->command.position;
    double effort = goal->command.max_effort;
    if(!math::nearly_equal_or_in_range(joint->limit_lower, joint->limit_upper, position))
    {
        setGoalState(actionlib::SimpleClientGoalState::StateEnum::ABORTED, false, false,
            "joint [" + joint_name + "]'s target position [" + std::to_string(position) + "] is not in bound[" + std::to_string(joint->limit_lower) + ":" + std::to_string(joint->limit_upper) + "].");
        return false;
    }
    if(!math::nearly_equal_or_in_range(-joint->limit_effort, joint->limit_effort, effort))
    {
        setGoalState(actionlib::SimpleClientGoalState::StateEnum::ABORTED, false, false,
            "joint [" + joint_name + "]'s max_effort [" + std::to_string(effort) + "] is not in bound[" + std::to_string(-joint->limit_effort) + ":" + std::to_string(joint->limit_effort) + "].");
        return false;
    }
    return true;
}

bool ToroboGripperController::initJointStateMap(control_msgs::GripperCommandGoalConstPtr goal)
{
    jointstate_map_.clear();

    // Validate goal's target position and effort
    if(!validateGoal(goal))
    {
        return false;
    }

    State state;
    state.goal = goal->command.position;
    state.goal_tolerance = GOAL_TOLERANCE_DEFAULT;
    std::string joint_name = getJoint(0)->name;
    jointstate_map_[joint_name] = state;

    return true;
}

control_msgs::GripperCommandResult ToroboGripperController::createResult(bool stalled, bool reached_goal)
{
    control_msgs::GripperCommandResult res;
    res.stalled = stalled;
    res.reached_goal = reached_goal;
    if(jointstate_map_.empty())
    {
        res.position = 0.0;
        res.effort = 0.0;
    }
    else
    {
        res.position = jointstate_map_.begin()->second.position;
        res.effort = jointstate_map_.begin()->second.effort;
    }
    return res;
}

} // namespace torobo
