/**
 * @file  torobo_controller_base.cpp
 * @brief Torobo controller base class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "torobo_driver/torobo_controller_base.h"

using namespace std;

namespace torobo
{

static const map<string, int> _paramNameNumberMap = 
{
    { "kp", 1 },
    { "ki", 2 },
    { "kd", 3 },
    { "wl", 4 },
    { "dyna_coef", 10 },
    { "dyna_intercept", 11 },
    { "static_coef", 12 },
    { "damping_eft_th", 13 },
    { "torque_const", 20 },
    { "eft_th", 21 },
    { "gripper_max_effort", 50 }, { "grippermaxeffort", 50 },
    { "velocity_override", 70 }, { "velov", 70 }, { "override", 70 },
    { "velocity_max", 71 }, { "velmax", 71 },
    { "acceleration_max", 72 }, { "accmax", 72 },
    { "jerk_max", 73 }, { "jerkmax", 73 },
    { "gravity_torque", 200 }, { "gravity_effort", 200 },
    { "dynamics_torque_cur", 201 }, { "dynamics_effort_cur", 201 },
    { "dynamics_torque_ref", 202 }, { "dynamics_effort_ref", 202 },
    { "ex_torque_cur", 203 }, { "ex_effort_cur", 203 },
    { "ex_torque_ref", 204 }, { "ex_effort_ref", 204 },
    { "out_conv_in_position", 300 }, { "outConvInPosition", 300 }, { "in_position", 300 },
    { "in_enc_position", 301 }, 
    { "in_velocity", 302 }, 
    { "in_acceleration", 303 }, 
    { "out_enc_position", 304 }, 
    { "power_current", 305 }, 
    { "power_consumption", 306 }, 
};

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
ToroboControllerBase::ToroboControllerBase(ros::NodeHandle &nh,
                                           const std::shared_ptr<MasterControllerClient>& client,
                                           const std::string& controllerName,
                                           const ToroboDriverCommonParam& common_param,
                                           const ControllerParam& controller_param,
                                           const int allJointsNum,
                                           bool debug)
    : nh_(nh),
      client_(client),
      common_param_(common_param),
      controller_name_(controllerName),
      controller_param_(controller_param),
      all_joints_num_(allJointsNum),
      debug_(debug)
{
    if(controller_param_.joint_names.size() < 1)
    {
        return;
    }

    // Initialize jointNameMap
    for(int i = 0; i < controller_param_.joint_names.size(); i++)
    {
        string jointName = controller_param_.joint_names[i];
        int jointId = controller_param_.joint_ids[i];
        std::string jointNameNumStr = to_string(jointId + 1); 
        joints_name_id_map_.insert(make_pair(jointName, jointId));
        joints_name_num_str_id_map_.insert(make_pair(jointNameNumStr, jointId));
    }

    // Initialize Joint State
    InitializeJointState();

    // Create controller independent send buffer for MCC
    client_->CreateIndependentSendPacketBuffer(controller_name_);
}

ToroboControllerBase::~ToroboControllerBase()
{
    joints_name_id_map_.clear();
    joints_name_num_str_id_map_.clear();

    // shutdown subscribers
    for(auto itr = subs_.begin(); itr != subs_.end(); ++itr)
    {
        itr->shutdown();
    }
    // shutdown service servers
    for(auto itr = srvs_.begin(); itr != srvs_.end(); ++itr)
    {
        itr->shutdown();
    }
}

void ToroboControllerBase::SpinOnce()
{
    sub_cb_queue_.callAvailable(ros::WallDuration(0));
    srv_cb_queue_.callAvailable(ros::WallDuration(0));
}

void ToroboControllerBase::Publish(const RecvPacket& packet, const ros::Time rosTimeStamp)
{
    for(auto itr = pubs_.begin(); itr != pubs_.end(); ++itr)
    {
        (*itr)->Publish(packet, rosTimeStamp);
    }
}

void ToroboControllerBase::PrintParam()
{
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "Controller:" <<  std::endl;
    std::cout << "  Name: " <<  controller_name_ << std::endl;
    std::cout << "  JointsNum : " << joints_name_id_map_.size() << std::endl;
    for(auto itr = joints_name_id_map_.begin(); itr != joints_name_id_map_.end(); ++itr)
    {
        std::cout << "    " << itr->first << ", " << itr->second << std::endl;
    }
    std::cout << "-----------------------------------------" << std::endl;
}

void ToroboControllerBase::SetJointState(const RecvPacket& recvPacket)
{
    const int s = (int)recvPacket.m_joint.size();
    const int jointsNum = (int)joints_name_id_map_.size();
    for(int i = 0; i < s; i++)
    {
        joint_state_.position[i] = recvPacket.m_joint[i].position;
        joint_state_.velocity[i] = recvPacket.m_joint[i].velocity;
        joint_state_.effort[i] = recvPacket.m_joint[i].effort;
    }
}

void ToroboControllerBase::SubsribeToroboDynamicsCallback(const torobo_msgs::ToroboDynamics::ConstPtr& msg)
{
    SendPacket sendPacket = NewSendPacket();
    for(int i = 0; i < msg->name.size(); i++)
    {
        string jointName = msg->name[i];
        if(joints_name_id_map_.count(jointName) == 0)
        {
            continue;
        }
        string idstr = ParseJointNameStringToIdString(jointName);
        sendPacket.SetDynamics(idstr,
                               (float)msg->gravity_compensation_effort[i],
                               (float)msg->ref_dynamics_effort[i],
                               (float)msg->cur_dynamics_effort[i],
                               (float)msg->inertia_diagonal[i]
                              );
    }
    client_->PushBackIndependentSendBuffer(sendPacket, controller_name_, false, true, "dynamics");
}

void ToroboControllerBase::SubsribeToroboCommandCallback(const torobo_msgs::ToroboCommand::ConstPtr& msg)
{
    SendPacket sendPacket = NewSendPacket();
    ePacketArmOrder wholeOrder = (ePacketArmOrder)msg->wholeOrder;
    sendPacket.SetArmOrder(wholeOrder);

    for(int i = 0; i < msg->name.size(); i++)
    {
        string jointName = msg->name[i];
        if(joints_name_id_map_.count(jointName) == 0)
        {
            continue;
        }
        string idstr = ParseJointNameStringToIdString(jointName);
        ePacketOrder jointOrder = (ePacketOrder)msg->jointOrder[i];
        float value1 = msg->value1[i];
        float value2 = msg->value2[i];
        float value3 = msg->value3[i];
        float value4 = msg->value4[i];
        sendPacket.SetCommonCommand(idstr, jointOrder, value1, value2, value3, value4);
    }

    if(debug_)
    {
        ROS_INFO("Send Common Command");
        ROS_INFO("WholeOrder = %d", sendPacket.m_preData.armOrder);
        for(int i = 0; i < all_joints_num_; i++)
        {
            ROS_INFO("    [%d, %d, %f, %f, %f, %f, ]",
             sendPacket.m_joint[i].ID,
             sendPacket.m_joint[i].jointOrder,
             sendPacket.m_joint[i].value1,
             sendPacket.m_joint[i].value2,
             sendPacket.m_joint[i].value3,
             sendPacket.m_joint[i].value4
            );
        }
    }
    client_->PushBackIndependentSendBuffer(sendPacket, controller_name_);
}

bool ToroboControllerBase::BrakeOffService(torobo_msgs::BrakeOff::Request &req, torobo_msgs::BrakeOff::Response &res)
{
    if(debug_)
    {
        ROS_INFO("[%s] brake off service is called", controller_name_.c_str());
    }
    res.success = SendBySendPacketMethod(req.joint_names, &SendPacket::SetBrakeOff);
    return true;
}

bool ToroboControllerBase::BrakeOnService(torobo_msgs::BrakeOn::Request &req, torobo_msgs::BrakeOn::Response &res)
{
    if(debug_)
    {
        ROS_INFO("[%s] brake on service is called", controller_name_.c_str());
    }
    res.success = SendBySendPacketMethod(req.joint_names, &SendPacket::SetBrakeOn);
    return true;
}

bool ToroboControllerBase::ServoOffService(torobo_msgs::ServoOff::Request &req, torobo_msgs::ServoOff::Response &res)
{
    if(debug_)
    {
        ROS_INFO("[%s] servo off service is called", controller_name_.c_str());
    }
    res.success = SendBySendPacketMethod(req.joint_names, &SendPacket::SetServoOff);
    return true;
}

bool ToroboControllerBase::ServoOnService(torobo_msgs::ServoOn::Request &req, torobo_msgs::ServoOn::Response &res)
{
    if(debug_)
    {
        ROS_INFO("[%s] servo on service is called", controller_name_.c_str());
    }
    res.success = SendBySendPacketMethod(req.joint_names, &SendPacket::SetServoOn);
    return true;
}

bool ToroboControllerBase::GetServoStateService(torobo_msgs::GetServoState::Request &req, torobo_msgs::GetServoState::Response &res)
{
    if(debug_)
    {
        ROS_INFO("[%s] get servo state service is called", controller_name_.c_str());
    }
    vector<uint8_t> idVec = ParseJointNameStringToIdVector(req.joint_names);
    if(idVec.size() == 0)
    {
        return false;
    }
    const RecvPacket recvPacket = client_->GetLastRecvPacket();
    for(auto itr = idVec.begin(); itr != idVec.end(); ++itr)
    {
        uint8_t id = *itr;
        if(id >= all_joints_num_)
        {
            continue;
        }
        const uint8_t systemMode = recvPacket.m_joint[id].systemMode;
        const bool isServoOn = systemMode == 2 ? true : false;
        res.is_servo_on.push_back(isServoOn);
    }
    return true;
}

bool ToroboControllerBase::ErrorResetService(torobo_msgs::ErrorReset::Request &req, torobo_msgs::ErrorReset::Response &res)
{
    if(debug_)
    {
        ROS_INFO("[%s] error reset service is called", controller_name_.c_str());
    }
    res.success = SendBySendPacketMethod(req.joint_names, &SendPacket::SetReset);
    return true;
}

bool ToroboControllerBase::SetControlModeService(torobo_msgs::SetControlMode::Request &req, torobo_msgs::SetControlMode::Response &res)
{
    if(debug_)
    {
        ROS_INFO("[%s] set control mode service is called", controller_name_.c_str());
    }
    void (SendPacket::*setMethod)(const std::string&);
    std::string controlModeName = req.control_mode_name.c_str();
    if(debug_)
    {
        ROS_INFO("set control mode name: %s", controlModeName.c_str());
    }

    if(controlModeName == "pos" || controlModeName == "position" || controlModeName == "traj" || controlModeName == "trajectory" )
    {
        setMethod = &SendPacket::SetTrajectoryControlMode;
    }
    else if(controlModeName == "vel" || controlModeName == "velocity")
    {
        setMethod = &SendPacket::SetVelocityControlMode;
    }
    else if(controlModeName == "cur" || controlModeName == "current")
    {
        setMethod = &SendPacket::SetCurrentControlMode;
    }
    else if(controlModeName == "extff" || controlModeName == "external_force_following")
    {
        setMethod = &SendPacket::SetExternalForceFollowingControlMode;
    }
    else if(controlModeName == "ontraj" || controlModeName == "online_trajectory")
    {
        setMethod = &SendPacket::SetOnlineTrajectoryControlMode;
    }
    else
    {
        res.success = false;
        return false;
    }
    res.success = SendBySendPacketMethod(req.joint_names, setMethod);
    return true;
}

bool ToroboControllerBase::SetZeroEffortService(torobo_msgs::SetZeroEffort::Request &req, torobo_msgs::SetZeroEffort::Response &res)
{
    if(debug_)
    {
        ROS_INFO("[%s] set zero effort service is called", controller_name_.c_str());
    }
    res.success = SendBySendPacketMethod(req.joint_names, &SendPacket::SetGetSlaveEffortOffset);
    return true;
}

bool ToroboControllerBase::ClearTrajectoryService(torobo_msgs::ClearTrajectory::Request &req, torobo_msgs::ClearTrajectory::Response &res)
{
    if(debug_)
    {
        ROS_INFO("[%s] clear trajectory service is called", controller_name_.c_str());
    }
    res.success = SendBySendPacketMethod(req.joint_names, &SendPacket::SetTrajectoryViaClear);
    return true;
}

bool ToroboControllerBase::CancelTrajectoryService(torobo_msgs::CancelTrajectory::Request &req, torobo_msgs::CancelTrajectory::Response &res)
{
    if(debug_)
    {
        ROS_INFO("[%s] cancel trajectory service is called", controller_name_.c_str());
    }
    res.success = SendBySendPacketMethod(req.joint_names, &SendPacket::SetTrajectoryControlCancel);
    return true;
}

bool ToroboControllerBase::SetRobotControllerParameterService(torobo_msgs::SetRobotControllerParameter::Request &req, torobo_msgs::SetRobotControllerParameter::Response &res)
{
    if(debug_)
    {
        ROS_INFO("[%s] set robot controller parameter service is called", controller_name_.c_str());
    }
    vector<string> idstrVec = ParseJointNameStringToIdStringVector(req.joint_names);
    vector<float> paramVec;
    if(req.joint_names.size() == 1 && req.joint_names[0] == "all" && req.parameter_values.size() == 1)
    {
        paramVec = vector<float>(idstrVec.size(), req.parameter_values[0]);
    }
    else
    {
        paramVec = vector<float>(req.parameter_values);
    }
    if(idstrVec.size() == 0 || idstrVec.size() != paramVec.size())
    {
        res.success = false;
        return false;
    }

    void (SendPacket::*setMethod)(const std::string&, const float);
    int arbitraryParamNumber = 0;
    std::string paramName = req.parameter_name.c_str();
    if(paramName == "kp")
    {
        setMethod = &SendPacket::SetParamKp;
    }
    else if(paramName == "ki")
    {
        setMethod = &SendPacket::SetParamKi;
    }
    else if(paramName == "kd")
    {
        setMethod = &SendPacket::SetParamKd;
    }
    else if(paramName == "wl" || paramName == "windup" || paramName == "windup_limit")
    {
        setMethod = &SendPacket::SetParamWindupLimit;
    }
    else if(paramName == "velocity_override")
    {
        setMethod = &SendPacket::SetParamVelocityOverride;
    }
    else if(paramName == "velocity_max")
    {
        setMethod = &SendPacket::SetParamVelocityMax;
    }
    else if(paramName == "acceleration_max")
    {
        setMethod = &SendPacket::SetParamAccelerationMax;
    }
    else if(paramName == "jerk_max")
    {
        setMethod = &SendPacket::SetParamJerkMax;
    }
    else if(_paramNameNumberMap.count(paramName) > 0)
    {
        arbitraryParamNumber = _paramNameNumberMap.at(paramName);
    }
    else
    {
        ROS_ERROR("given parameter name: %s is unknown", paramName.c_str());
        res.success = false;
        return false;
    }
    if(debug_)
    {
        ROS_INFO("set parameter name : %s", paramName.c_str());
    }

    SendPacket sendPacket = NewSendPacket();
    for(int i = 0; i < idstrVec.size(); i++)
    {
        if(arbitraryParamNumber != 0)
        {
            sendPacket.SetArbitraryParam(idstrVec[i], arbitraryParamNumber, paramVec[i]);
        }
        else
        {
            (sendPacket.*setMethod)(idstrVec[i], paramVec[i]);
        }
    }
    client_->PushBackIndependentSendBuffer(sendPacket, controller_name_);
    res.success = true;
    return true;
}

bool ToroboControllerBase::SetGeneralOutputRegisterService(torobo_msgs::SetGeneralOutputRegister::Request &req, torobo_msgs::SetGeneralOutputRegister::Response &res)
{
    if(debug_)
    {
        ROS_INFO("[%s] set general output register service is called", controller_name_.c_str());
    }
    const int general_register_number = req.general_register_number;
    vector<string> idstrVec = ParseJointNameStringToIdStringVector(req.joint_names);
    if(idstrVec.size() == 0)
    {
        ROS_ERROR("empty valid joint_names is given");
        res.success = false;
        return false;
    }
    else if((general_register_number < 0) || (general_register_number > 3))
    {
        ROS_ERROR("invalid target general register number is given");
        res.success = false;
        return false;
    }

    std::string paramName = req.parameter_name.c_str();
    int paramNumber = 0;
    if(_paramNameNumberMap.count(paramName) > 0)
    {
        paramNumber = _paramNameNumberMap.at(paramName);
    }
    else
    {
        ROS_ERROR("given parameter name: %s is unknown", paramName.c_str());
        res.success = false;
        return false;
    }
    if(debug_)
    {
        ROS_INFO("set general register number [%d], parameter name: %s", general_register_number, paramName.c_str());
    }

    SendPacket sendPacket = NewSendPacket();
    for(int i = 0; i < idstrVec.size(); i++)
    {
        sendPacket.SetCommonCommand(idstrVec[i], (ePacketOrder)224, general_register_number, paramNumber);
    }
    client_->PushBackIndependentSendBuffer(sendPacket, controller_name_);
    res.success = true;
    return true;
}

bool ToroboControllerBase::SetPayloadParamService(torobo_msgs::SetPayloadParam::Request &req, torobo_msgs::SetPayloadParam::Response &res)
{
    if(debug_)
    {
        ROS_INFO("[%s] set payload param service is called", controller_name_.c_str());
    }

    const double mass = req.mass;
    double massCenterX = 0.0f;
    double massCenterY = 0.0f;
    double massCenterZ = 0.0f;
    if(req.com.size() == 3)
    {
        massCenterX = req.com[0];
        massCenterY = req.com[1];
        massCenterZ = req.com[2];
    }
    SendPacket sendPacket = NewSendPacket();
    sendPacket.SetPayload(mass, massCenterX, massCenterY, massCenterZ);
    client_->PushBackIndependentSendBuffer(sendPacket, controller_name_);

    if(debug_)
    {
        ROS_INFO("set payload mass : %f, massCenter = [%f, %f, %f]", mass, massCenterX, massCenterY, massCenterZ);
    }

    if(req.inertiaElem.size() == 6)
    {
        const double ixx = req.inertiaElem[0];
        const double ixy = req.inertiaElem[1];
        const double ixz = req.inertiaElem[2];
        const double iyy = req.inertiaElem[3];
        const double iyz = req.inertiaElem[4];
        const double izz = req.inertiaElem[5];

        sendPacket = NewSendPacket();
        sendPacket.SetPayloadInertiaTensorDiagonal(ixx, iyy, izz);
        client_->PushBackIndependentSendBuffer(sendPacket, controller_name_);

        sendPacket = NewSendPacket();
        sendPacket.SetPayloadInertiaTensorTriangle(ixy, ixz, iyz);
        client_->PushBackIndependentSendBuffer(sendPacket, controller_name_);
    }

    res.success = true;
    return true;
}

bool ToroboControllerBase::SendCommonCommandService(torobo_msgs::SendCommonCommand::Request &req, torobo_msgs::SendCommonCommand::Response &res)
{
    if(debug_)
    {
        ROS_INFO("[%s] send common command service is called", controller_name_.c_str());
    }
    string idstr = ParseJointNameStringToIdString(req.joint_names);
    if(idstr == "")
    {
        res.success = false;
        return false;
    }

    SendPacket sendPacket = NewSendPacket();
    ePacketArmOrder wholeOrder = (ePacketArmOrder)req.whole_order;
    ePacketOrder jointOrder = (ePacketOrder)req.joint_order;
    float value1 = req.value1;
    float value2 = req.value2;
    float value3 = req.value3;
    float value4 = req.value4;
    sendPacket.SetArmOrder(wholeOrder);
    sendPacket.SetCommonCommand(idstr, jointOrder, value1, value2, value3, value4);

    client_->PushBackIndependentSendBuffer(sendPacket, controller_name_);
    if(debug_)
    {
        ROS_INFO("Send Common Command = [%s, %d, %d, %f, %f, %f, %f, ]", idstr.c_str(), wholeOrder, jointOrder, value1, value2, value3, value4);
    }

    res.success = true;
    return true;
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
void ToroboControllerBase::InitializeJointState()
{
    joint_state_.name.resize(all_joints_num_);
    joint_state_.position.resize(all_joints_num_);
    joint_state_.velocity.resize(all_joints_num_);
    joint_state_.effort.resize(all_joints_num_);
    for(const auto& itr : joints_name_id_map_)
    {
        const string& joint_name = itr.first;
        const int id = itr.second;
        joint_state_.name[id] = joint_name;
    }
}

void ToroboControllerBase::PushBackPublisher(std::unique_ptr<AbstractPublisher>&& pub)
{
    pubs_.push_back(std::move(pub));
}

std::string ToroboControllerBase::ParseJointNameStringToIdString(const std::string& jointName) const
{
    vector<string> jointNameVec;
    jointNameVec.push_back(jointName);
    return ParseJointNameStringToIdString(jointNameVec);
}

std::string ToroboControllerBase::ParseJointNameStringToIdString(const std::vector<std::string>& jointNameVec) const
{
    string idstr = "";
    size_t s = jointNameVec.size();

    if(s < 1)
    {
        return "";
    }

    if(jointNameVec[0] == "all")
    {
        for(auto itr = joints_name_id_map_.begin(); itr != joints_name_id_map_.end(); ++itr)
        {
            string id = to_string(itr->second);
            idstr += (itr == joints_name_id_map_.begin()) ? id : "/" + id;
        }
        return idstr;
    }

    for(int i = 0; i < s; i++)
    {
        string id = "";
        string name = jointNameVec[i];
        if(joints_name_id_map_.count(name) > 0)
        {
            id = to_string(joints_name_id_map_.at(name));
        }
        else if(joints_name_num_str_id_map_.count(name) > 0)
        {
            id = to_string(joints_name_num_str_id_map_.at(name));
        }
        else
        {
            return "";
        }

        idstr += (i == 0) ? id : "/" + id;
    }
    return idstr;
}

std::vector<std::string> ToroboControllerBase::ParseJointNameStringToIdStringVector(const std::string& jointName) const
{
    vector<string> jointNameVec;
    jointNameVec.push_back(jointName);
    return ParseJointNameStringToIdStringVector(jointNameVec);
}

std::vector<std::string> ToroboControllerBase::ParseJointNameStringToIdStringVector(const std::vector<std::string>& jointNameVec) const
{
    vector<string> idstrVec;
    size_t s = jointNameVec.size();

    if(s < 1)
    {
        return idstrVec;
    }

    if(jointNameVec[0] == "all")
    {
        for(auto itr = joints_name_id_map_.begin(); itr != joints_name_id_map_.end(); ++itr)
        {
            string id = to_string(itr->second);
            idstrVec.push_back(id);
        }
        return idstrVec;
    }

    for(int i = 0; i < s; i++)
    {
        string id = "";
        string name = jointNameVec[i];
        if(joints_name_id_map_.count(name) > 0)
        {
            id = to_string(joints_name_id_map_.at(name));
        }
        else if(joints_name_num_str_id_map_.count(name) > 0)
        {
            id = to_string(joints_name_num_str_id_map_.at(name));
        }
        else
        {
            return idstrVec;
        }

        idstrVec.push_back(id);
    }
    return idstrVec;
}

std::vector<uint8_t> ToroboControllerBase::ParseJointNameStringToIdVector(const std::string& jointName) const
{
    vector<string> jointNameVec;
    jointNameVec.push_back(jointName);
    return ParseJointNameStringToIdVector(jointNameVec);
}

std::vector<uint8_t> ToroboControllerBase::ParseJointNameStringToIdVector(const std::vector<std::string>& jointNameVec) const
{
    vector<uint8_t> idVec;
    size_t s = jointNameVec.size();

    if(s < 1)
    {
        return idVec;
    }

    if(jointNameVec[0] == "all")
    {
        for(auto itr = joints_name_id_map_.begin(); itr != joints_name_id_map_.end(); ++itr)
        {
            uint8_t id = itr->second;
            idVec.push_back(id);
        }
        return idVec;
    }

    for(int i = 0; i < s; i++)
    {
        uint8_t id = 0;
        string name = jointNameVec[i];
        if(joints_name_id_map_.count(name) > 0)
        {
            id = joints_name_id_map_.at(name);
        }
        else if(joints_name_num_str_id_map_.count(name) > 0)
        {
            id = joints_name_num_str_id_map_.at(name);
        }
        else
        {
            return idVec;
        }

        idVec.push_back(id);
    }
    return idVec;
}

SendPacket ToroboControllerBase::NewSendPacket() const
{
    return SendPacket(all_joints_num_);
}

bool ToroboControllerBase::SendBySendPacketMethod(std::vector<std::string> jointNames, void (SendPacket::*setMethod)(const std::string&))
{
    string idstr = ParseJointNameStringToIdString(jointNames);
    // ROS_INFO("  target id:[%s]", idstr.c_str());
    if(idstr == "")
    {
        return false;
    }

    SendPacket sendPacket = NewSendPacket();
    (sendPacket.*setMethod)(idstr);
    client_->PushBackIndependentSendBuffer(sendPacket, controller_name_);
    return true;
}

}
