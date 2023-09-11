/**
 * @file  torobo_joint_controller.h
 * @brief Torobo joint controller class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_JOINT_CONTROLLER_H
#define TOROBO_JOINT_CONTROLLER_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include "torobo_driver/torobo_controller_base.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboJointController : public ToroboControllerBase
{
public:
    ToroboJointController(ros::NodeHandle &nh,
                          const std::shared_ptr<MasterControllerClient>& client,
                          const std::string& controllerName,
                          const ToroboDriverCommonParam& common_param,
                          const ControllerParam& controller_param,
                          const int allJointsNum,
                          bool debug = false);
    virtual ~ToroboJointController();

protected:
    void InitializePublisher();
    void InitializeSubscriber();
    void InitializeServiceServer();
    bool isNearEqualJointPose(const trajectory_msgs::JointTrajectoryPoint& point, const std::vector<int> idVec, const double epsilon) const;

    void SubsribeJointTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
    void SendTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint& point, const std::vector<std::string>& idstrVec);
};

}

#endif
