/**
 * @file  torobo_controller_base.h
 * @brief Torobo controller base class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_CONTROLLER_BASE_H
#define TOROBO_CONTROLLER_BASE_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <memory>
#include <thread>
#include <sensor_msgs/JointState.h>
#include "MasterControllerClient/MasterControllerClient.h"
#include "MasterControllerClient/Communication/Packet/RecvPacket.h"
#include "torobo_driver/abstract_publisher.h"
#include "torobo_driver/torobo_driver_param.h"
#include "torobo_msgs/ToroboDynamics.h"
#include "torobo_msgs/ToroboCommand.h"
#include "torobo_msgs/ErrorReset.h"
#include "torobo_msgs/BrakeOff.h"
#include "torobo_msgs/BrakeOn.h"
#include "torobo_msgs/ServoOff.h"
#include "torobo_msgs/ServoOn.h"
#include "torobo_msgs/GetServoState.h"
#include "torobo_msgs/SetControlMode.h"
#include "torobo_msgs/SetZeroEffort.h"
#include "torobo_msgs/ClearTrajectory.h"
#include "torobo_msgs/CancelTrajectory.h"
#include "torobo_msgs/SetRobotControllerParameter.h"
#include "torobo_msgs/SetGeneralOutputRegister.h"
#include "torobo_msgs/SetPayloadParam.h"
#include "torobo_msgs/SendCommonCommand.h"

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboControllerBase
{
public:
    ToroboControllerBase(ros::NodeHandle &nh,
                         const std::shared_ptr<MasterControllerClient>& client,
                         const std::string& controllerName,
                         const ToroboDriverCommonParam& common_param,
                         const ControllerParam& controller_param,
                         const int allJointsNum,
                         bool debug = false);
    virtual ~ToroboControllerBase();

    void SpinOnce();
    void Publish(const RecvPacket& packet, const ros::Time rosTimeStamp);
    SendPacketBuffer& GetSendPacketBuffer();
    void PrintParam();
    void SetJointState(const RecvPacket& recvPacket);

    void SubsribeToroboDynamicsCallback(const torobo_msgs::ToroboDynamics::ConstPtr& msg);
    void SubsribeToroboCommandCallback(const torobo_msgs::ToroboCommand::ConstPtr& msg);

    bool BrakeOffService(torobo_msgs::BrakeOff::Request &req, torobo_msgs::BrakeOff::Response &res);
    bool BrakeOnService(torobo_msgs::BrakeOn::Request &req, torobo_msgs::BrakeOn::Response &res);
    bool ServoOffService(torobo_msgs::ServoOff::Request &req, torobo_msgs::ServoOff::Response &res);
    bool ServoOnService(torobo_msgs::ServoOn::Request &req, torobo_msgs::ServoOn::Response &res);
    bool GetServoStateService(torobo_msgs::GetServoState::Request &req, torobo_msgs::GetServoState::Response &res);
    bool ErrorResetService(torobo_msgs::ErrorReset::Request &req, torobo_msgs::ErrorReset::Response &res);
    bool SetControlModeService(torobo_msgs::SetControlMode::Request &req, torobo_msgs::SetControlMode::Response &res);
    bool SetZeroEffortService(torobo_msgs::SetZeroEffort::Request &req, torobo_msgs::SetZeroEffort::Response &res);
    bool ClearTrajectoryService(torobo_msgs::ClearTrajectory::Request &req, torobo_msgs::ClearTrajectory::Response &res);
    bool CancelTrajectoryService(torobo_msgs::CancelTrajectory::Request &req, torobo_msgs::CancelTrajectory::Response &res);
    bool SetRobotControllerParameterService(torobo_msgs::SetRobotControllerParameter::Request &req, torobo_msgs::SetRobotControllerParameter::Response &res);
    bool SetGeneralOutputRegisterService(torobo_msgs::SetGeneralOutputRegister::Request &req, torobo_msgs::SetGeneralOutputRegister::Response &res);
    bool SetPayloadParamService(torobo_msgs::SetPayloadParam::Request &req, torobo_msgs::SetPayloadParam::Response &res);
    bool SendCommonCommandService(torobo_msgs::SendCommonCommand::Request &req, torobo_msgs::SendCommonCommand::Response &res);
protected:
    void InitializeJointState();
    void PushBackPublisher(std::unique_ptr<AbstractPublisher>&& pub);

    std::string ParseJointNameStringToIdString(const std::string& jointName) const;
    std::string ParseJointNameStringToIdString(const std::vector<std::string>& jointNameVec) const;
    std::vector<std::string> ParseJointNameStringToIdStringVector(const std::string& jointName) const;
    std::vector<std::string> ParseJointNameStringToIdStringVector(const std::vector<std::string>& jointNameVec) const;
    std::vector<uint8_t> ParseJointNameStringToIdVector(const std::string& jointName) const;
    std::vector<uint8_t> ParseJointNameStringToIdVector(const std::vector<std::string>& jointNameVec) const;
    SendPacket NewSendPacket() const;
    bool SendBySendPacketMethod(std::vector<std::string> jointNames, void (SendPacket::*setMethod)(const std::string&));

    bool debug_;
    ros::NodeHandle& nh_;
    std::shared_ptr<MasterControllerClient> client_;
    const int all_joints_num_;
    std::map<std::string, int> joints_name_id_map_;
    std::map<std::string, int> joints_name_num_str_id_map_;
    const std::string controller_name_;
    const torobo::ToroboDriverCommonParam common_param_;
    const torobo::ControllerParam controller_param_;

    sensor_msgs::JointState joint_state_;

    std::vector<std::unique_ptr<AbstractPublisher>> pubs_;
    std::vector<ros::Subscriber> subs_;
    std::vector<ros::ServiceServer> srvs_;
    ros::CallbackQueue sub_cb_queue_;
    ros::CallbackQueue srv_cb_queue_;
};

}

#endif
