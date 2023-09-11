/**
 * @file  torobo_action_controller_base.h
 * @brief ToroboActionControllerBase class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_ACTION_CONTROLLER_BASE_H
#define TOROBO_ACTION_CONTROLLER_BASE_H


/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <torobo_msgs/ToroboJointState.h>
#include <torobo_msgs/CancelTrajectory.h>
#include <torobo_common/math_util.h>


namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboActionControllerBase
{
public:
    struct Joint
    {
        std::string name;
        double limit_lower = 0.0;
        double limit_upper = 0.0;
        double limit_effort = 0.0;
        double limit_velocity = 0.0;
        Joint(std::string name_value, double limit_lower_value, double limit_upper_value, double limit_effort_value, double limit_velocity_value)
        {
            name = name_value;
            limit_lower = limit_lower_value;
            limit_upper = limit_upper_value;
            limit_effort = limit_effort_value;
            limit_velocity = limit_velocity_value;
        }
    };
    struct State
    {
        double goal = 0.0;
        double goal_tolerance = 0.0;
        double position = 0.0;
        double velocity = 0.0;
        double effort = 0.0;
        int trjStatus = 0;
    };
    static const int TRAJ_STATUS_RUNNING = 2;
    static const int TRAJ_STATUS_COMPLETE = 4;
    static const int TRAJ_STATUS_CANCEL_COMPLETE = 6;

    ToroboActionControllerBase(ros::NodeHandle& node, std::string name, std::string action_name);
    virtual ~ToroboActionControllerBase();

    void registerJoint(std::string joint_name, double limit_lower, double limit_upper, double limit_effort, double limit_velocity);

protected:
    // access to action controller's joint property
    bool hasJoint(std::string joint_name);
    std::shared_ptr<Joint> getJoint(std::string joint_name);
    std::shared_ptr<Joint> getJoint(int index);

    // access to action controller's state property
    bool initTimeVariables(double time_from_start, double goal_time_tolerance, double additional_duration);
    bool updateJointstateMap(const torobo_msgs::ToroboJointState::ConstPtr& msg);
    bool checkTimeNowIsInMonitoringLimit();
    bool checkForTrajectoryRunning(bool immediate_complete_is_possible = false);
    bool checkForTrajectoryComplete();
    bool waitForTrajectoryCancelComplete(double timeout);
    bool checkGoalIsInTolerance();
    bool checkGoalTimeIsInTolerance();
    std::string getJointStateString();

    // call cancel trajectory service
    bool callCancelTrajectoryService(ros::ServiceClient& service_client);

    // create child NodeHandle in order to separate parent node's thread
    std::unique_ptr<ros::NodeHandle> nh_ptr_;
    ros::CallbackQueue callback_queue_;
    std::unique_ptr<ros::AsyncSpinner> async_spinner_ptr_;

    // action controller's property
    std::string name_;
    std::string action_name_;
    std::vector<std::shared_ptr<Joint> > joints_;

    // action controller's state variables
    std::map<std::string, State> jointstate_map_;
    bool trj_status_running_ = false;

    // action controller's time variables
    ros::Time start_time_ = ros::Time();
    ros::Time goal_time_ = ros::Time();
    ros::Duration goal_time_tolerance_ = ros::Duration(0.0);
    ros::Duration additional_monitoring_duration_ = ros::Duration(0.0);

};

} // namespace torobo

#endif /* TOROBO_ACTION_CONTROLLER_BASE_H */

