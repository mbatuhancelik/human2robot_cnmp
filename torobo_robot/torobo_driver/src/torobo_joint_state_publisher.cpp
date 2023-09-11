/**
 * @file  torobo_joint_state_publisher.cpp
 * @brief Joint trajectory controller state publisher class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "torobo_driver/torobo_joint_state_publisher.h"

using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
ToroboJointStatePublisher::ToroboJointStatePublisher(ros::NodeHandle &nh,
                                         const std::string& controller_name,
                                         const std::map<std::string, int>& jointsNameIdMap,
                                         const ToroboJointStatePublisher::Coef& coef,
                                         const int queueSize)
    : AbstractPublisher(nh, controller_name, jointsNameIdMap),
      coef_(coef)
{
    Initialize(queueSize);
}

ToroboJointStatePublisher::~ToroboJointStatePublisher()
{
}

void ToroboJointStatePublisher::Publish(const RecvPacket& packet, const ros::Time& rosTimeStamp)
{
    torobo_joint_state_.header.stamp = rosTimeStamp;
    torobo_joint_state_.timeStamp = packet.m_preData.timeStamp;
    torobo_joint_state_.hostTimeStamp = packet.m_preData.hostTimeStamp;

    int idx = 0;
    for(auto itr = joints_name_id_map_.begin(); itr != joints_name_id_map_.end(); ++itr)
    {
        const int i = itr->second;
        torobo_joint_state_.type[idx] = packet.m_joint[i].type;
        torobo_joint_state_.comStatus[idx] = packet.m_joint[i].comStatus;
        torobo_joint_state_.systemMode[idx] = packet.m_joint[i].systemMode;
        torobo_joint_state_.ctrlMode[idx] = packet.m_joint[i].ctrlMode;
        torobo_joint_state_.errorWarningStatus[idx] = packet.m_joint[i].ewStatus;
        torobo_joint_state_.trjStatus[idx] = packet.m_joint[i].trjStatus;
        torobo_joint_state_.trjViaRemain[idx] = packet.m_joint[i].trjViaRemain;
        torobo_joint_state_.refCurrent[idx] = packet.m_joint[i].refCurrent;
        torobo_joint_state_.refPosition[idx] = packet.m_joint[i].refPosition * coef_.position;
        torobo_joint_state_.refVelocity[idx] = packet.m_joint[i].refVelocity * coef_.velocity;
        torobo_joint_state_.refAcceleration[idx] = packet.m_joint[i].refAcceleration * coef_.acceleration;
        torobo_joint_state_.refEffort[idx] = packet.m_joint[i].refEffort * coef_.effort;
        torobo_joint_state_.current[idx] = packet.m_joint[i].current;
        torobo_joint_state_.position[idx] = packet.m_joint[i].position * coef_.position;
        torobo_joint_state_.velocity[idx] = packet.m_joint[i].velocity * coef_.velocity;
        torobo_joint_state_.acceleration[idx] = packet.m_joint[i].acceleration * coef_.acceleration;
        torobo_joint_state_.outConvInVelocity[idx] = packet.m_joint[i].outConvInVelocity * coef_.velocity;
        torobo_joint_state_.outConvInAcceleration[idx] = packet.m_joint[i].outConvInAcceleration * coef_.acceleration;
        torobo_joint_state_.effort[idx] = packet.m_joint[i].effort * coef_.effort;
        torobo_joint_state_.temperature[idx] = packet.m_joint[i].temperature;
        torobo_joint_state_.general_0[idx] = packet.m_joint[i].general[0];
        torobo_joint_state_.general_1[idx] = packet.m_joint[i].general[1];
        torobo_joint_state_.general_2[idx] = packet.m_joint[i].general[2];
        torobo_joint_state_.general_3[idx] = packet.m_joint[i].general[3];
        idx++;
    }
    pub_.publish(torobo_joint_state_);
}




/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
void ToroboJointStatePublisher::Initialize(const int queueSize)
{
    torobo_joint_state_.header.stamp = ros::Time::now();
    torobo_joint_state_.timeStamp = 0;
    torobo_joint_state_.hostTimeStamp = 0;
    for(auto itr = joints_name_id_map_.begin(); itr != joints_name_id_map_.end(); ++itr)
    {
        torobo_joint_state_.name.push_back(itr->first);
        torobo_joint_state_.type.push_back(0);
        torobo_joint_state_.comStatus.push_back(0);
        torobo_joint_state_.systemMode.push_back(0);
        torobo_joint_state_.ctrlMode.push_back(0);
        torobo_joint_state_.errorWarningStatus.push_back(0);
        torobo_joint_state_.trjStatus.push_back(0);
        torobo_joint_state_.trjViaRemain.push_back(0);
        torobo_joint_state_.refCurrent.push_back(0.0);
        torobo_joint_state_.refPosition.push_back(0.0);
        torobo_joint_state_.refVelocity.push_back(0.0);
        torobo_joint_state_.refAcceleration.push_back(0.0);
        torobo_joint_state_.refEffort.push_back(0.0);
        torobo_joint_state_.current.push_back(0.0);
        torobo_joint_state_.position.push_back(0.0);
        torobo_joint_state_.velocity.push_back(0.0);
        torobo_joint_state_.acceleration.push_back(0.0);
        torobo_joint_state_.outConvInVelocity.push_back(0.0);
        torobo_joint_state_.outConvInAcceleration.push_back(0.0);
        torobo_joint_state_.effort.push_back(0.0);
        torobo_joint_state_.temperature.push_back(0.0);
        torobo_joint_state_.general_0.push_back(0.0);
        torobo_joint_state_.general_1.push_back(0.0);
        torobo_joint_state_.general_2.push_back(0.0);
        torobo_joint_state_.general_3.push_back(0.0);
    }
    pub_ = nh_.advertise<torobo_msgs::ToroboJointState>(controller_name_ + "/torobo_joint_state", queueSize, true);
}


}
