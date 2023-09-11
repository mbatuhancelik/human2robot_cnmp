/**
 * @file  torobo_joint_controller.cpp
 * @brief Torobo joint controller class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "torobo_driver/torobo_joint_controller.h"
#include "torobo_driver/joint_state_publisher.h"
#include "torobo_driver/joint_trajectory_controller_state_publisher.h"
#include "torobo_driver/torobo_joint_state_publisher.h"
#include <torobo_common/math_util.h>
#include <angles/angles.h>

using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
ToroboJointController::ToroboJointController(ros::NodeHandle &node,
                                             const std::shared_ptr<MasterControllerClient>& client,
                                             const std::string& controllerName,
                                             const ToroboDriverCommonParam& common_param,
                                             const ControllerParam& controller_param,
                                             const int allJointsNum,
                                             bool debug)
    : ToroboControllerBase(node, client, controllerName, common_param, controller_param, allJointsNum, debug)
{
    // Initialize pusb/subs/srvs
    InitializePublisher();
    InitializeSubscriber();
    InitializeServiceServer();
}

ToroboJointController::~ToroboJointController()
{
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
void ToroboJointController::InitializePublisher()
{
    const double deg2rad = angles::from_degrees(1.0);

    unique_ptr<AbstractPublisher> pub;
    pub.reset(new JointStatePublisher(nh_, controller_name_, joints_name_id_map_, JointStatePublisher::Coef(deg2rad, deg2rad, 1.0), 1));
    PushBackPublisher(move(pub));

    pub.reset(new JointTrajectoryControllerStatePublisher(nh_, controller_name_, joints_name_id_map_, JointTrajectoryControllerStatePublisher::Coef(deg2rad, deg2rad, deg2rad, 1.0), 1));
    PushBackPublisher(move(pub));

    pub.reset(new ToroboJointStatePublisher(nh_, controller_name_, joints_name_id_map_, ToroboJointStatePublisher::Coef(deg2rad, deg2rad, deg2rad, 1.0), 1));
    PushBackPublisher(move(pub));
}

void ToroboJointController::InitializeSubscriber()
{
    ros::SubscribeOptions dyna_so = ros::SubscribeOptions::create<torobo_msgs::ToroboDynamics>(
        controller_name_ + "/torobo_dynamics",
        1,
        boost::bind(&ToroboControllerBase::SubsribeToroboDynamicsCallback, static_cast<ToroboControllerBase*>(this), _1),
        ros::VoidPtr(),
        &sub_cb_queue_ 
    );
    dyna_so.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true),
    subs_.emplace_back(nh_.subscribe(dyna_so));

    ros::SubscribeOptions tc_so = ros::SubscribeOptions::create<torobo_msgs::ToroboCommand>(
        controller_name_ + "/torobo_command",
        1,
        boost::bind(&ToroboControllerBase::SubsribeToroboCommandCallback, static_cast<ToroboControllerBase*>(this), _1),
        ros::VoidPtr(),
        &sub_cb_queue_ 
    );
    tc_so.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true),
    subs_.emplace_back(nh_.subscribe(tc_so));

    ros::SubscribeOptions jt_so = ros::SubscribeOptions::create<trajectory_msgs::JointTrajectory>(
        controller_name_ + "/command",
        1,
        boost::bind(&ToroboJointController::SubsribeJointTrajectoryCallback, this, _1),
        ros::VoidPtr(),
        &sub_cb_queue_ 
    );
    jt_so.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true),
    subs_.emplace_back(nh_.subscribe(jt_so));
}

void ToroboJointController::InitializeServiceServer()
{
    ros::AdvertiseServiceOptions brake_off_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::BrakeOff>(
        controller_name_ + "/brake_off",
        boost::bind(&ToroboControllerBase::BrakeOffService, static_cast<ToroboControllerBase*>(this), _1, _2),
        ros::VoidPtr(), 
        &srv_cb_queue_
    );
    srvs_.emplace_back(nh_.advertiseService(brake_off_aso));

    ros::AdvertiseServiceOptions brake_on_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::BrakeOn>(
        controller_name_ + "/brake_on",
        boost::bind(&ToroboControllerBase::BrakeOnService, static_cast<ToroboControllerBase*>(this), _1, _2),
        ros::VoidPtr(), 
        &srv_cb_queue_
    );
    srvs_.emplace_back(nh_.advertiseService(brake_on_aso));

    ros::AdvertiseServiceOptions servo_off_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::ServoOff>(
        controller_name_ + "/servo_off",
        boost::bind(&ToroboControllerBase::ServoOffService, static_cast<ToroboControllerBase*>(this), _1, _2),
        ros::VoidPtr(), 
        &srv_cb_queue_
    );
    srvs_.emplace_back(nh_.advertiseService(servo_off_aso));

    ros::AdvertiseServiceOptions servo_on_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::ServoOn>(
        controller_name_ + "/servo_on",
        boost::bind(&ToroboControllerBase::ServoOnService, static_cast<ToroboControllerBase*>(this), _1, _2),
        ros::VoidPtr(), 
        &srv_cb_queue_
    );
    srvs_.emplace_back(nh_.advertiseService(servo_on_aso));

    ros::AdvertiseServiceOptions get_servo_state_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::GetServoState>(
        controller_name_ + "/get_servo_state",
        boost::bind(&ToroboControllerBase::GetServoStateService, static_cast<ToroboControllerBase*>(this), _1, _2),
        ros::VoidPtr(), 
        &srv_cb_queue_
    );
    srvs_.emplace_back(nh_.advertiseService(get_servo_state_aso));

    ros::AdvertiseServiceOptions error_reset_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::ErrorReset>(
        controller_name_ + "/error_reset",
        boost::bind(&ToroboControllerBase::ErrorResetService, static_cast<ToroboControllerBase*>(this), _1, _2),
        ros::VoidPtr(), 
        &srv_cb_queue_
    );
    srvs_.emplace_back(nh_.advertiseService(error_reset_aso));

    ros::AdvertiseServiceOptions set_control_mode_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::SetControlMode>(
        controller_name_ + "/set_control_mode",
        boost::bind(&ToroboControllerBase::SetControlModeService, static_cast<ToroboControllerBase*>(this), _1, _2),
        ros::VoidPtr(), 
        &srv_cb_queue_
    );
    srvs_.emplace_back(nh_.advertiseService(set_control_mode_aso));

    ros::AdvertiseServiceOptions set_zero_effort_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::SetZeroEffort>(
        controller_name_ + "/set_zero_effort",
        boost::bind(&ToroboControllerBase::SetZeroEffortService, static_cast<ToroboControllerBase*>(this), _1, _2),
        ros::VoidPtr(), 
        &srv_cb_queue_
    );
    srvs_.emplace_back(nh_.advertiseService(set_zero_effort_aso));

    ros::AdvertiseServiceOptions clear_trajectory_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::ClearTrajectory>(
        controller_name_ + "/clear_trajectory",
        boost::bind(&ToroboControllerBase::ClearTrajectoryService, static_cast<ToroboControllerBase*>(this), _1, _2),
        ros::VoidPtr(), 
        &srv_cb_queue_
    );
    srvs_.emplace_back(nh_.advertiseService(clear_trajectory_aso));

    ros::AdvertiseServiceOptions cancel_trajectory_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::CancelTrajectory>(
        controller_name_ + "/cancel_trajectory",
        boost::bind(&ToroboControllerBase::CancelTrajectoryService, static_cast<ToroboControllerBase*>(this), _1, _2),
        ros::VoidPtr(), 
        &srv_cb_queue_
    );
    srvs_.emplace_back(nh_.advertiseService(cancel_trajectory_aso));

    ros::AdvertiseServiceOptions set_robot_controller_parameter_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::SetRobotControllerParameter>(
        controller_name_ + "/set_robot_controller_parameter",
        boost::bind(&ToroboControllerBase::SetRobotControllerParameterService, static_cast<ToroboControllerBase*>(this), _1, _2),
        ros::VoidPtr(), 
        &srv_cb_queue_
    );
    srvs_.emplace_back(nh_.advertiseService(set_robot_controller_parameter_aso));

    ros::AdvertiseServiceOptions set_general_output_register_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::SetGeneralOutputRegister>(
        controller_name_ + "/set_general_output_register",
        boost::bind(&ToroboControllerBase::SetGeneralOutputRegisterService, static_cast<ToroboControllerBase*>(this), _1, _2),
        ros::VoidPtr(), 
        &srv_cb_queue_
    );
    srvs_.emplace_back(nh_.advertiseService(set_general_output_register_aso));

    ros::AdvertiseServiceOptions send_common_command_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::SendCommonCommand>(
        controller_name_ + "/send_common_command",
        boost::bind(&ToroboControllerBase::SendCommonCommandService, static_cast<ToroboControllerBase*>(this), _1, _2),
        ros::VoidPtr(), 
        &srv_cb_queue_
    );
    srvs_.emplace_back(nh_.advertiseService(send_common_command_aso));

    if(controller_name_ == "arm_controller")
    {
        ros::AdvertiseServiceOptions set_payload_param_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::SetPayloadParam>(
            controller_name_ + "/set_payload_param",
            boost::bind(&ToroboControllerBase::SetPayloadParamService, static_cast<ToroboControllerBase*>(this), _1, _2),
            ros::VoidPtr(), 
            &srv_cb_queue_
        );
        srvs_.emplace_back(nh_.advertiseService(set_payload_param_aso));
    }
}

void ToroboJointController::SubsribeJointTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
    // ROS_INFO("Subscribe JointTrajectory. length:%d", (int)msg->points.size());

    // Clear all via points
    SendBySendPacketMethod(msg->joint_names, &SendPacket::SetTrajectoryViaClear);

    // Make idstrVec
    vector<string> idstrVec;
    vector<int> idVec;
    for(auto itr = msg->joint_names.begin(); itr != msg->joint_names.end(); ++itr)
    {
        const string jointName = *itr;
        if(joints_name_id_map_.count(jointName) == 0)
        {
            ROS_ERROR("Commanded points have invalid joint names --> Failed to run trajectory.");
            return;
        }
        int id = joints_name_id_map_.at(jointName);
        string idstr = to_string(id);
        idstrVec.push_back(idstr);
        idVec.push_back(id);
    }

    // Send trajectory points
    int sendPointsNum = 0;
    const int s = (int)msg->points.size();
    for(int i = 0; i < s; i++)
    {
        // skip t == 0 point if position near equal current position
        if(i == 0)
        {
            const float t = msg->points[0].time_from_start.toSec();
            if(math::nearly_equal(0.0, t, 1e-4) &&
              isNearEqualJointPose(msg->points[i], idVec, common_param_.allowed_start_tolerance))
            {
                // skip
                continue;
            }
        }
        SendTrajectoryPoint(msg->points[i], idstrVec);
        sendPointsNum++;
    }

    if(sendPointsNum == 0)
    {
        ROS_WARN("Commanded trajectory points is invalid! --> Any trajectory points is not send.");
        return;
    }

    // Start trajectory control
    SendBySendPacketMethod(msg->joint_names, &SendPacket::SetTrajectoryControlStart);
}

void ToroboJointController::SendTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint& point, const std::vector<std::string>& idstrVec)
{
    const double rad2deg = angles::to_degrees(1.0);
    SendPacket sendPacket = NewSendPacket();
    bool uniqueSendPacketFlag = false;

    const size_t posSize = point.positions.size();
    const size_t velSize = point.velocities.size();
    const size_t accSize = point.accelerations.size();

    for(size_t i = 0; i < posSize; i++)
    {
        const float t = point.time_from_start.sec + point.time_from_start.nsec * 1e-9;
        const float pos = float(point.positions[i] * rad2deg);
        float vel = 0.0f;
        if(i < velSize)
        {
            vel = float(point.velocities[i] * rad2deg);
        }
#if 1
        sendPacket.SetTrajectoryPVT(idstrVec[i], pos, vel, t);
        // sendPacket.SetTrajectoryPT_ContinuousSpline(idstrVec[i], pos, t);
        // ROS_INFO("Send TrajectoryPoint[P,V,T] = [%f, %f, %f]", pos, vel, t);

        const RecvPacket recvPacket = client_->GetLastRecvPacket();
        const uint8_t ctrlMode = recvPacket.m_joint[i].ctrlMode;
        if(ctrlMode == 6 || ctrlMode == 7)
        {
            uniqueSendPacketFlag = true;
        }
#else
        float acc = 0.0f;
        if(i < velSize)
        {
            acc = float(point.accelerations[i] * rad2deg);
        }
        sendPacket.SetTrajectoryPVAT(idstrVec[i], pos, vel, acc, t);
        ROS_INFO("Send TrajectoryPoint[P,V,A,T] = [%f, %f, %f, %f]", pos, vel, acc, t);
#endif
    }
    if(uniqueSendPacketFlag)
    {
        client_->PushBackIndependentSendBuffer(sendPacket, controller_name_, false, true, "pvt");
    }
    else
    {
        client_->PushBackIndependentSendBuffer(sendPacket, controller_name_);
    }
}

bool ToroboJointController::isNearEqualJointPose(const trajectory_msgs::JointTrajectoryPoint& point, const std::vector<int> idVec, const double epsilon) const
{
    for(int i = 0; i < idVec.size(); i++)
    {
        const int id = idVec[i];
        const double current_pos_rad = angles::from_degrees(joint_state_.position[id]);
        const double diff = fabs(point.positions[i] - current_pos_rad);
        if(diff > epsilon)
        {
            ROS_ERROR("[Torobo Driver]\nInvalid Trajectory: start point deviates from current robot state more than %g"
                      "\njoint '%s': expected: %g, current: %g",
                      epsilon, joint_state_.name[id].c_str(), point.positions[id], current_pos_rad);
            return false;
        }
    }
    return true;
}

}
