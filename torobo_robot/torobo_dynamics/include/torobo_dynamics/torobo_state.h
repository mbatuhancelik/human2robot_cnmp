/**
 * @file  ToroboState.h
 * @brief Torobo state class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_STATE_H
#define TOROBO_STATE_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <string>
#include <map>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "torobo_msgs/ToroboJointState.h"
#include "torobo_msgs/ToroboDynamics.h"

namespace torobo
{

class JointState;

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboState
{
public:
    ToroboState(ros::NodeHandle& node, const std::map<std::string, std::vector<std::string>>& controller_joints_map, bool sim);
    virtual ~ToroboState();

    const JointState* GetToroboJointState(std::string name) const;
    void Publish();

protected:
    void InitializeToroboJointStateMap(const std::string& prefix, const std::vector<std::string>& joints);
    void UpdateJointState(const sensor_msgs::JointState::ConstPtr& msg);

    ros::NodeHandle& node_;
    std::map<std::string, ros::Subscriber> torobo_state_sub_;
    std::map<std::string, ros::Publisher> torobo_state_pub_;
    ros::Subscriber joint_state_sub_;
    std::map<std::string, ros::Publisher> torobo_dynamics_pub_;

    std::map<std::string, std::vector<std::string>> controller_joints_map_;
    std::map<std::string, JointState> state_;
    std::map<std::string, torobo_msgs::ToroboJointState> toroboJointStateMap_;

    ros::Time last_time_;
    std::map<std::string, double> last_position_;
    std::map<std::string, double> last_velocity_;

    std::map<std::string, torobo_msgs::ToroboDynamics> dynamics_;
    bool sim_;

private:
};

}

#endif
