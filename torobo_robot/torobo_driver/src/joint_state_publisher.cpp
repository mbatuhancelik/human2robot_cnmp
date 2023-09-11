/**
 * @file  joint_state_publisher.cpp
 * @brief Joint state publisher class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "torobo_driver/joint_state_publisher.h"

using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
JointStatePublisher::JointStatePublisher(ros::NodeHandle& nh,
                                         const std::string& controller_name,
                                         const std::map<std::string, int>& jointsNameIdMap,
                                         const JointStatePublisher::Coef& coef,
                                         const int queueSize)
    : AbstractPublisher(nh, controller_name, jointsNameIdMap),
      coef_(coef)
{
    Initialize(queueSize);
}

JointStatePublisher::~JointStatePublisher()
{
}

void JointStatePublisher::Publish(const RecvPacket& packet, const ros::Time& rosTimeStamp)
{
    joint_state_.header.stamp = rosTimeStamp;
    int idx = 0;
    for(auto itr = joints_name_id_map_.begin(); itr != joints_name_id_map_.end(); ++itr)
    {
        const int i = itr->second;
        joint_state_.position[idx] = packet.m_joint[i].position * coef_.position;
        joint_state_.velocity[idx] = packet.m_joint[i].velocity * coef_.velocity;
        joint_state_.effort[idx] = packet.m_joint[i].effort * coef_.effort;
        idx++;
    }
    pub_.publish(joint_state_);
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
void JointStatePublisher::Initialize(const int queueSize)
{
    joint_state_.header.stamp = ros::Time::now();
    for(auto itr = joints_name_id_map_.begin(); itr != joints_name_id_map_.end(); ++itr)
    {
        joint_state_.name.push_back(itr->first);
        joint_state_.position.push_back(0.0);
        joint_state_.velocity.push_back(0.0);
        joint_state_.effort.push_back(0.0);
    }
    pub_ = nh_.advertise<sensor_msgs::JointState>(controller_name_ + "/joint_state", queueSize, true);
}

}
