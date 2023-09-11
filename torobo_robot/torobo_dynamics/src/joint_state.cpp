/**
 * @file  JointState.h
 * @brief Joint state class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "torobo_dynamics/joint_state.h"

// #define MEASURE_TIME
#ifdef MEASURE_TIME
#include <chrono>
#include <iostream>
static std::chrono::time_point<std::chrono::system_clock> t0;
#endif

using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Privat Global Variables
 ----------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
JointState::JointState(int jointsNum, std::string name)
{
    jointsNum_ = jointsNum;
    name_ = name;

    for(int i = 0; i < jointsNum_; i++)
    {
        joint_names_.push_back("");
        position_.push_back(0.0);
        velocity_.push_back(0.0);
        ref_velocity_.push_back(0.0);
        acceleration_.push_back(0.0);
        ref_acceleration_.push_back(0.0);
        effort_.push_back(0.0);
        current_.push_back(0.0);
    }
}

JointState::~JointState()
{
}

int JointState::GetJointsNum()
{
    return jointsNum_;
}

void JointState::UpdateState(const torobo_msgs::ToroboJointState::ConstPtr& msg)
{
    int size = (int)msg->name.size();
    if(size > jointsNum_)
    {
        size = jointsNum_;
    }
    for(int i = 0; i < size; i++)
    {
        joint_names_[i] = msg->name[i];
        current_[i] = msg->current[i];
        position_[i] = msg->position[i];
#if 1
        velocity_[i] = msg->outConvInVelocity[i];
        acceleration_[i] = msg->outConvInAcceleration[i];
#else
        velocity_[i] = msg->velocity[i];
        acceleration_[i] = msg->acceleration[i];
#endif
        ref_velocity_[i] = msg->refVelocity[i];
        ref_acceleration_[i] = msg->refAcceleration[i];
        effort_[i] = msg->effort[i];
    }

#ifdef MEASURE_TIME
    auto t = chrono::system_clock::now();
    auto usec = chrono::duration_cast<chrono::microseconds>(t - t0).count();
    t0 = t;
    std::cout << "[torobo_dynamics][" << name_.c_str() << "]Time: " << usec << std::endl;
#endif
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/

}
