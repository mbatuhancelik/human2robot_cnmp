/**
 * @file  ToroboGripperController.cpp
 * @brief Torobo joint controller class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "torobo_driver/torobo_gripper_controller.h"
#include "torobo_driver/joint_state_publisher.h"
#include "torobo_driver/joint_trajectory_controller_state_publisher.h"
#include "torobo_driver/torobo_joint_state_publisher.h"
#include <angles/angles.h>
#include <memory>

using namespace std;

namespace torobo
{

const double millimeter2meter = 1.0 / 1000.0;
const double meter2millimeter = 1000.0;
double master2ros = 1.0;
double ros2master = 1.0;

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
ToroboGripperController::ToroboGripperController(ros::NodeHandle &nh,
                                                 const std::shared_ptr<MasterControllerClient>& client,
                                                 const std::string& controllerName,
                                                 const ToroboDriverCommonParam& common_param,
                                                 const ControllerParam& controller_param,
                                                 const int allJointsNum,
                                                 bool debug)
    : ToroboControllerBase(nh, client, controllerName, common_param, controller_param, allJointsNum, debug)
{
    // Initialize pusb/subs/srvs
    InitializePublisher();
    InitializeSubscriber();
    InitializeServiceServer();
}

ToroboGripperController::~ToroboGripperController()
{
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
void ToroboGripperController::InitializePublisher()
{
    if(controller_param_.joint_types[0] == "revolute")
    {
        master2ros = angles::from_degrees(1.0);
        ros2master = angles::to_degrees(1.0);
    }
    else if(controller_param_.joint_types[0] == "prismatic")
    {
        master2ros = millimeter2meter;
        ros2master = meter2millimeter;
    }

    unique_ptr<AbstractPublisher> pub;
    pub.reset(new JointStatePublisher(nh_, controller_name_, joints_name_id_map_, JointStatePublisher::Coef(master2ros, master2ros, 1.0), 1));
    PushBackPublisher(move(pub));

    pub.reset(new JointTrajectoryControllerStatePublisher(nh_, controller_name_, joints_name_id_map_, JointTrajectoryControllerStatePublisher::Coef(master2ros, master2ros, master2ros, 1.0), 1));
    PushBackPublisher(move(pub));

    pub.reset(new ToroboJointStatePublisher(nh_, controller_name_, joints_name_id_map_, ToroboJointStatePublisher::Coef(master2ros, master2ros, master2ros, 1.0), 1));
    PushBackPublisher(move(pub));
}

void ToroboGripperController::InitializeSubscriber()
{
    ros::SubscribeOptions tc_so = ros::SubscribeOptions::create<torobo_msgs::ToroboCommand>(
        controller_name_ + "/torobo_command",
        1,
        boost::bind(&ToroboControllerBase::SubsribeToroboCommandCallback, static_cast<ToroboControllerBase*>(this), _1),
        ros::VoidPtr(),
        &sub_cb_queue_ 
    );
    tc_so.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true),
    subs_.emplace_back(nh_.subscribe(tc_so));

    ros::SubscribeOptions gc_so = ros::SubscribeOptions::create<control_msgs::GripperCommand>(
        controller_name_ + "/command",
        1,
        boost::bind(&ToroboGripperController::SubsribeGripperCommandCallback, this, _1),
        ros::VoidPtr(),
        &sub_cb_queue_ 
    );
    gc_so.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true),
    subs_.emplace_back(nh_.subscribe(gc_so));
}

void ToroboGripperController::InitializeServiceServer()
{
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
}

void ToroboGripperController::SubsribeJointTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
    if(debug_)
    {
        ROS_INFO("Subscribe JointTrajectory. length:%d", (int)msg->points.size());
    }

    // Clear all via points
    SendBySendPacketMethod(msg->joint_names, &SendPacket::SetTrajectoryViaClear);

    // Make idstrVec
    vector<string> idstrVec;
    for(auto itr = msg->joint_names.begin(); itr != msg->joint_names.end(); ++itr)
    {
        string jointName = *itr;
        int id = joints_name_id_map_.at(jointName);
        string idstr = to_string(id);
        idstrVec.push_back(idstr);
    }

    // Send trajectory points
    for(auto itr = msg->points.begin(); itr != msg->points.end(); ++itr)
    {
        SendTrajectoryPoint(*itr, idstrVec);
    }

    // Start trajectory control
    SendBySendPacketMethod(msg->joint_names, &SendPacket::SetTrajectoryControlStart);
}

void ToroboGripperController::SubsribeGripperCommandCallback(const control_msgs::GripperCommand::ConstPtr& msg)
{
    if(debug_)
    {
        ROS_INFO("Subscribe GripperCommand. position:%f, max_effort:%f", msg->position, msg->max_effort);
    }

    const RecvPacket recvPacket = client_->GetLastRecvPacket();
    const int gripper_id = joints_name_id_map_.begin()->second;
    const uint8_t ctrlMode = recvPacket.m_joint[gripper_id].ctrlMode;
    
    if(ctrlMode == 2)   // Current control mode
    {
        // Gripper close
        if(msg->position < 0.01f)
        {
            SendGripperCurrent(-0.5f);
        }
        // Gripper open
        else if(msg->position > 0.01f)
        {
            SendGripperCurrent(0.5f);
        }
    }
    else
    {
        SendGripperMaxEffort(msg->max_effort);
        SendGripperPosition(msg->position);
    }
    
}

void ToroboGripperController::SendTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint point, const std::vector<std::string> idstrVec)
{
    SendPacket sendPacket = NewSendPacket();
    for(int i = 0; i < point.positions.size(); i++)
    {
        const float t = point.time_from_start.sec + point.time_from_start.nsec * 1e-9;
        const float pos = float(point.positions[i] * ros2master);
        const float vel = float(point.velocities[i] * ros2master);
        const float acc = float(point.accelerations[i] * ros2master);
        sendPacket.SetTrajectoryPVT(idstrVec[i], pos, vel, t);
        if(debug_)
        {
            ROS_INFO("Send TrajectoryPoint[P,V,T] = [%f, %f, %f]", pos, vel, t);
        }
    }
    client_->PushBackIndependentSendBuffer(sendPacket, controller_name_);
}

void ToroboGripperController::SendGripperMaxEffort(const double gripperMaxEffort)
{
    SendPacket sendPacket = NewSendPacket();
    sendPacket.SetGripperMaxEffort((float)gripperMaxEffort);
    client_->PushBackIndependentSendBuffer(sendPacket, controller_name_);
}

void ToroboGripperController::SendGripperPosition(const double position)
{
    SendPacket sendPacket = NewSendPacket();
    string idstr = to_string(joints_name_id_map_.begin()->second);
    const float pos = (float)(position * ros2master);
    sendPacket.SetRefPosition(idstr, pos);
    client_->PushBackIndependentSendBuffer(sendPacket, controller_name_);
}

void ToroboGripperController::SendGripperCurrent(const double current)
{
    SendPacket sendPacket = NewSendPacket();
    string idstr = to_string(joints_name_id_map_.begin()->second);
    sendPacket.SetRefCurrent(idstr, current);
    client_->PushBackIndependentSendBuffer(sendPacket, controller_name_);
}

}
