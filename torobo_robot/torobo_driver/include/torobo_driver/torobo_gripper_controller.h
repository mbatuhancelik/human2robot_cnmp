/**
 * @file  torobo_gripper_controller.h
 * @brief Torobo gripper controller class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_GRIPPER_CONTROLLER_H
#define TOROBO_GRIPPER_CONTROLLER_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include "torobo_driver/torobo_controller_base.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/GripperCommand.h>

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboGripperController : public ToroboControllerBase
{
public:
    ToroboGripperController(ros::NodeHandle &nh,
                            const std::shared_ptr<MasterControllerClient>& client,
                            const std::string& controllerName,
                            const ToroboDriverCommonParam& common_param,
                            const ControllerParam& controller_param,
                            const int allJointsNum,
                            bool debug=false);
    virtual ~ToroboGripperController();

protected:
    void InitializePublisher();
    void InitializeSubscriber();
    void InitializeServiceServer();

    void SubsribeJointTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
    void SubsribeGripperCommandCallback(const control_msgs::GripperCommand::ConstPtr& msg);

    void SendTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint point, const std::vector<std::string> idstrVec);
    void SendGripperMaxEffort(const double gripperMaxEffort);
    void SendGripperPosition(const double position);
    void SendGripperCurrent(const double current);
};

}

#endif
