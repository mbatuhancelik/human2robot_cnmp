/**
 * @file  joint_trajectory_controller_state_publisher.cpp
 * @brief Joint trajectory controller state publisher class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "torobo_driver/joint_trajectory_controller_state_publisher.h"

using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
JointTrajectoryControllerStatePublisher::JointTrajectoryControllerStatePublisher(ros::NodeHandle &nh,
                                         const std::string& controller_name,
                                         const std::map<std::string, int>& jointsNameIdMap,
                                         const JointTrajectoryControllerStatePublisher::Coef& coef,
                                         const int queueSize)
    : AbstractPublisher(nh, controller_name, jointsNameIdMap),
      coef_(coef)
{
    Initialize(queueSize);
}

JointTrajectoryControllerStatePublisher::~JointTrajectoryControllerStatePublisher()
{
}

void JointTrajectoryControllerStatePublisher::Publish(const RecvPacket& packet, const ros::Time& rosTimeStamp)
{
    joint_trajectory_controller_state_.header.stamp = rosTimeStamp;
    int idx = 0;
    for(auto itr = joints_name_id_map_.begin(); itr != joints_name_id_map_.end(); ++itr)
    {
        const int i = itr->second;
        const double refpos = packet.m_joint[i].refPosition * coef_.position;
        const double refvel = packet.m_joint[i].refVelocity * coef_.velocity;
        const double refacc = packet.m_joint[i].refAcceleration * coef_.acceleration;
        const double refeft = packet.m_joint[i].refEffort * coef_.effort;
        const double curpos = packet.m_joint[i].position * coef_.position;
        const double curvel = packet.m_joint[i].velocity * coef_.velocity;
        const double curacc = packet.m_joint[i].acceleration * coef_.acceleration;
        const double cureft = packet.m_joint[i].effort * coef_.effort;
        const int32_t sec = (int32_t)packet.m_preData.duration;
        const int32_t nsec = (int32_t)(packet.m_preData.duration * 1000000000 - (int32_t)packet.m_preData.duration);
        joint_trajectory_controller_state_.desired.positions[idx] = refpos;
        joint_trajectory_controller_state_.desired.velocities[idx] = refvel;
        joint_trajectory_controller_state_.desired.accelerations[idx] = refacc;
        joint_trajectory_controller_state_.desired.effort[idx] = refeft;
        joint_trajectory_controller_state_.desired.time_from_start.sec = sec;
        joint_trajectory_controller_state_.desired.time_from_start.nsec = nsec;
        joint_trajectory_controller_state_.actual.positions[idx] = curpos;
        joint_trajectory_controller_state_.actual.velocities[idx] = curvel;
        joint_trajectory_controller_state_.actual.accelerations[idx] = curacc;
        joint_trajectory_controller_state_.actual.effort[idx] = cureft;
        joint_trajectory_controller_state_.actual.time_from_start.sec = sec;
        joint_trajectory_controller_state_.actual.time_from_start.nsec = nsec;
        joint_trajectory_controller_state_.error.positions[idx] = refpos - curpos;
        joint_trajectory_controller_state_.error.velocities[idx] = refvel - curvel;
        joint_trajectory_controller_state_.error.accelerations[idx] = refacc - curacc;
        joint_trajectory_controller_state_.error.effort[idx] = refeft - cureft;
        joint_trajectory_controller_state_.error.time_from_start.sec = sec;
        joint_trajectory_controller_state_.error.time_from_start.nsec = nsec;
        idx++;
    }
    pub_.publish(joint_trajectory_controller_state_);
}


/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
void JointTrajectoryControllerStatePublisher::Initialize(const int queueSize)
{
    joint_trajectory_controller_state_.header.stamp = ros::Time::now();
    for(auto itr = joints_name_id_map_.begin(); itr != joints_name_id_map_.end(); ++itr)
    {
        joint_trajectory_controller_state_.joint_names.push_back(itr->first);
        joint_trajectory_controller_state_.desired.positions.push_back(0.0);
        joint_trajectory_controller_state_.desired.velocities.push_back(0.0);
        joint_trajectory_controller_state_.desired.accelerations.push_back(0.0);
        joint_trajectory_controller_state_.desired.effort.push_back(0.0);
        joint_trajectory_controller_state_.actual.positions.push_back(0.0);
        joint_trajectory_controller_state_.actual.velocities.push_back(0.0);
        joint_trajectory_controller_state_.actual.accelerations.push_back(0.0);
        joint_trajectory_controller_state_.actual.effort.push_back(0.0);
        joint_trajectory_controller_state_.error.positions.push_back(0.0);
        joint_trajectory_controller_state_.error.velocities.push_back(0.0);
        joint_trajectory_controller_state_.error.accelerations.push_back(0.0);
        joint_trajectory_controller_state_.error.effort.push_back(0.0);
    }
    pub_ = nh_.advertise<control_msgs::JointTrajectoryControllerState>(controller_name_ + "/state", queueSize, true);
}

}
