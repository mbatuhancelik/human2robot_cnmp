/**
 * @file  JointState.h
 * @brief Joint state class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef JOINT_STATE_H
#define JOINT_STATE_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <vector>
#include <string>
#include "torobo_msgs/ToroboJointState.h"

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class JointState
{
public:
    JointState(int jointsNum, std::string name="");
    virtual ~JointState();

    int GetJointsNum();
    void UpdateState(const torobo_msgs::ToroboJointState::ConstPtr& msg);

    std::vector<std::string> joint_names_;
    std::vector<double> current_;
    std::vector<double> position_;
    std::vector<double> velocity_;
    std::vector<double> ref_velocity_;
    std::vector<double> acceleration_;
    std::vector<double> ref_acceleration_;
    std::vector<double> effort_;
protected:
    int jointsNum_;
    std::string name_;

private:
};

}

#endif
