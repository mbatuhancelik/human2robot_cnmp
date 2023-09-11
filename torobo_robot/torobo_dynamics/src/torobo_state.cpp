/**
 * @file  ToroboState.h
 * @brief Torobo state class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include "torobo_dynamics/torobo_state.h"
#include "torobo_dynamics/joint_state.h"

using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Privat Global Variables
 ----------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
ToroboState::ToroboState(ros::NodeHandle& node, const std::map<std::string, std::vector<std::string>>& controller_joints_map, bool sim)
 : node_(node), controller_joints_map_(controller_joints_map), sim_(sim)
{
    for(auto itr = controller_joints_map_.begin(); itr != controller_joints_map_.end(); ++itr)
    {
        const string& name = itr->first;
        const int jointsNum = itr->second.size();

        // Generate state subscriber
        state_.insert(make_pair(name, JointState(jointsNum, name)));
        JointState* const state_ptr = &(state_.at(name));
        ros::Subscriber sub = node_.subscribe(name + "/torobo_joint_state", 1, &JointState::UpdateState, state_ptr, ros::TransportHints().reliable().tcpNoDelay(true));
        torobo_state_sub_.insert(make_pair(name, sub));
    }
    if(sim_)
    {
        joint_state_sub_ = node_.subscribe("joint_states" , 1, &ToroboState::UpdateJointState, this);
        for(auto itr = controller_joints_map_.begin(); itr != controller_joints_map_.end(); ++itr)
        {
            string name = itr->first;
            InitializeToroboJointStateMap(name, itr->second);
            ros::Publisher pub  = node_.advertise<torobo_msgs::ToroboJointState>(name + "/torobo_joint_state", 1);
            torobo_state_pub_.insert(make_pair(name, pub));
        }
    }
}

ToroboState::~ToroboState()
{
}

const JointState* ToroboState::GetToroboJointState(std::string name) const
{
    if(state_.count(name) > 0)
    {
        return &(state_.at(name));
    }
    return NULL;
}

void ToroboState::Publish()
{
    if(!sim_)
    {
        return;
    }
    for(auto itr = toroboJointStateMap_.begin(); itr != toroboJointStateMap_.end(); ++itr)
    {
        string name = itr->first;
        if(torobo_state_pub_.count(name) == 0)
        {
            continue;
        }
        if(itr->second.name[0] == "")
        {
            continue;
        }

        itr->second.header.stamp = ros::Time::now();
        torobo_state_pub_.at(name).publish(itr->second);
    }
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
void ToroboState::InitializeToroboJointStateMap(const std::string& prefix, const std::vector<std::string>& joints)
{
    torobo_msgs::ToroboJointState toroboJointState;
    for(int i = 0; i < joints.size(); i++)
    {
        toroboJointState.name.push_back(joints[i]);
        toroboJointState.type.push_back(0);
        toroboJointState.comStatus.push_back(0);
        toroboJointState.systemMode.push_back(0);
        toroboJointState.ctrlMode.push_back(0);
        toroboJointState.errorWarningStatus.push_back(0);
        toroboJointState.trjStatus.push_back(0);
        toroboJointState.trjViaRemain.push_back(0);
        toroboJointState.refCurrent.push_back(0.0);
        toroboJointState.refPosition.push_back(0.0);
        toroboJointState.refVelocity.push_back(0.0);
        toroboJointState.refAcceleration.push_back(0.0);
        toroboJointState.refEffort.push_back(0.0);
        toroboJointState.current.push_back(0.0);
        toroboJointState.position.push_back(0.0);
        toroboJointState.velocity.push_back(0.0);
        toroboJointState.acceleration.push_back(0.0);
        toroboJointState.outConvInVelocity.push_back(0.0);
        toroboJointState.outConvInAcceleration.push_back(0.0);
        toroboJointState.effort.push_back(0.0);
        toroboJointState.temperature.push_back(0.0);
        toroboJointState.general_0.push_back(0.0);
        toroboJointState.general_1.push_back(0.0);
        toroboJointState.general_2.push_back(0.0);
        toroboJointState.general_3.push_back(0.0);
    }
    toroboJointStateMap_.insert(make_pair(prefix, toroboJointState));
}

void ToroboState::UpdateJointState(const sensor_msgs::JointState::ConstPtr& msg)
{
    const ros::Time t = ros::Time::now();
    const ros::Duration d = t - last_time_;
    const double dt = d.toSec();
    if(dt < 0.0)
    {
        return;
    }

    for(int i = 0; i < msg->name.size(); i++)
    {
        const string& name = msg->name[i];
        const double position = msg->position[i];

        for(auto itr = toroboJointStateMap_.begin(); itr != toroboJointStateMap_.end(); ++itr)
        {
            const vector<string>& joint_names = itr->second.name;
            for(auto jit = joint_names.begin(); jit < joint_names.end(); jit++)
            {
                if (name.find(*jit) == std::string::npos)
                {
                    continue;
                }
                const int idx = std::distance(joint_names.begin(), jit);

                itr->second.position[idx] = position;

                // estimate velocity & acceleration
                double velocity = 0.0;
                if(last_position_.count(name) > 0)
                {
                    velocity = (position - last_position_[name]) / dt;
                }

                double acceleration = 0.0;
                if(last_velocity_.count(name) > 0)
                {
                    acceleration = (velocity - last_velocity_[name]) / dt;
                }
                itr->second.velocity[idx] = velocity;
                itr->second.acceleration[idx] = acceleration;
                last_position_[name] = position;
                last_velocity_[name] = velocity;
            }
        }
    }
    last_time_ = t;
    Publish();
}

}
