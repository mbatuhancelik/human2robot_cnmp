/**
 * @file  torobo_driver.cpp
 * @brief Torobo driver class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "torobo_driver/torobo_driver.h"
#include "torobo_driver/torobo_joint_controller.h"
#include "torobo_driver/torobo_gripper_controller.h"
#include "MasterControllerClient/MasterControllerClientMock.h"
#include <chrono>

// #define MEASURE_TIME
#ifdef MEASURE_TIME
#include <iostream>
static std::chrono::time_point<std::chrono::system_clock> t0;
#endif

using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Private Functions
 ----------------------------------------------------------------------*/
std::string CastXmlRpcValueAsString(XmlRpc::XmlRpcValue value);
map<std::string, string> ParseXmlRpcStructToMap(XmlRpc::XmlRpcValue& value);

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
ToroboDriver::ToroboDriver(ros::NodeHandle &nh,
    const ToroboDriverCommonParam& common_param,
    const ToroboDriverParam& param,
    const std::chrono::nanoseconds& period_ns,
    const double timeout_sec,
    bool is_mock,
    bool debug
)
    : nh_(nh), common_param_(common_param),
    param_(param),
    period_ns_(period_ns),
    timeout_sec_(timeout_sec),
    is_mock_(is_mock),
    client_(nullptr),
    debug_(debug)
{
    init_ = false;
    thread_work_ = true;
}

ToroboDriver::~ToroboDriver()
{
}

void ToroboDriver::run()
{
    int64_t sleep_ms = period_ns_.count() / 1000000;
    if(sleep_ms <= 0)
    {
        sleep_ms = 1;
    }
    while(thread_work_)
    {
        runOnce();
        boost::this_thread::sleep(boost::posix_time::milliseconds(sleep_ms));
    }
}

void ToroboDriver::stop()
{
    thread_work_ = false;
}

void ToroboDriver::runOnce()
{
    if(nh_.ok())
    {
        SpinOnce();
        Publish();
        SendCommandToMaster();
    }
}

bool ToroboDriver::Initialize()
{
    if(init_)
    {
        return true;
    }

    // Create MasterControllerClient
    bool init = false;
    if(is_mock_)    // for debug/test
    {
        init = ConnectMock();
    }
    else if(param_.interface == "Ethernet")
    {
        init = ConnectEthernet(param_.ip, param_.port);
    }
    else if(param_.interface == "USB")
    {
        init = ConnectSerialPort(param_.com, param_.baudrate);
    }
    else
    {
        ROS_ERROR("interface[%s] is invalid", param_.interface.c_str());
    }

    if(!init)
    {
        init_ = false;
        ROS_ERROR("fail to connect");
        return false;
    }
    init_ = true;
    ROS_INFO("succeeded to connect");

    // Create controllers
    for(auto itr = param_.controller.begin(); itr != param_.controller.end(); ++itr)
    {
        string controllerName = itr->first;

        if(controllerName.find("gripper") != string::npos)
        {
            controllers_.emplace_back(new ToroboGripperController(nh_, client_, controllerName, common_param_, itr->second, param_.allJointsNum, debug_));
        }
        else
        {
            controllers_.emplace_back(new ToroboJointController(nh_, client_, controllerName, common_param_, itr->second, param_.allJointsNum, debug_));
        }
    }

    last_recv_time_ = ros::Time::now();
    return true;
}

bool ToroboDriver::IsInit()
{
    return init_;
}

void ToroboDriver::SpinOnce()
{
    for(auto citr = controllers_.begin(); citr != controllers_.end(); ++citr)
    {
        (*citr)->SpinOnce();
    }
}

void ToroboDriver::Publish()
{
    // Update client current status by receiving packets from master controller
    int32_t recvNum = client_->ReceiveStatus();
    // ROS_ERROR_STREAM("ToroboDriver[" << this << "] : publish thread id = " << std::this_thread::get_id());

    // Get current status vector
    std::vector<RecvPacket> curStateVec = client_->GetCurStatePacket();

    // Get current time
    ros::Time t = ros::Time::now();

    // Judge timeout & reconnect
    if(recvNum > 0)
    {
        last_recv_time_ = t;
    }
    else
    {
        if(param_.interface != "Ethernet")
        {
            return;
        }
        ros::Duration diff = t - last_recv_time_;
        if(diff.toSec() > timeout_sec_)
        {
            ROS_ERROR("Connection timeout. Try to re-connect...");
            bool ret = client_->ReConnect();
            if(!ret)
            {
                ROS_ERROR("Re-connect failed.");
                last_recv_time_ = t; // update last recv time for next re-connect after re-timeout
                return;
            }
            ROS_INFO("Succeeded to Re-connect.");
        }
        return;
    }

    // Publish status as a ROS message
#if 1
    // Publish latest state
    std::vector<RecvPacket>::iterator itr = curStateVec.end();
    --itr;
    for(auto citr = controllers_.begin(); citr != controllers_.end(); ++citr)
    {
        (*citr)->SetJointState(*itr);
        (*citr)->Publish(*itr, t);
    }
#else
    // Publish received all state
    for(std::vector<RecvPacket>::iterator itr = curStateVec.begin();
        itr != curStateVec.end(); ++itr)
    {
        for(auto citr = controllers_.begin(); citr != controllers_.end(); ++citr)
        {
            (*citr)->Publish(*itr, t);
        }
    }
#endif
}

void ToroboDriver::SendCommandToMaster()
{
    client_->SendPacketInBuffer();
#ifdef MEASURE_TIME
    auto now = chrono::system_clock::now();
    auto usec = chrono::duration_cast<chrono::microseconds>(now - t0).count();
    t0 = now;
    std::cout << "[torobo_driver]: " << param_.controller[0].first << ", Time: " << usec << std::endl;
#endif
}

bool ToroboDriver::ConnectEthernet(std::string ip, int port)
{
    client_.reset(new MasterControllerClient(param_.allJointsNum));
    ROS_INFO("Connect to MasterController as TCP client...");
    return client_->InitAsTcpClient(ip, port);
}

bool ToroboDriver::ConnectSerialPort(std::string comport, int baudrate)
{
    client_.reset(new MasterControllerClient(param_.allJointsNum));
    ROS_INFO("Connect to MasterController as SerialPort client...");
    return client_->InitAsSerialClient(comport, baudrate);
}

bool ToroboDriver::ConnectMock()
{
    client_.reset(new MasterControllerClientMock(param_.allJointsNum));
    ROS_INFO("Connect to MasterControllerClientMock");

    // Set joint types to mock
    shared_ptr<MasterControllerClientMock> mock = dynamic_pointer_cast<MasterControllerClientMock>(client_);
    if(!mock)
    {
        return true;
    }
    for(auto itr = param_.controller.begin(); itr != param_.controller.end(); ++itr)
    {
        string& controller_name = itr->first;
        for(int i = 0; i < itr->second.joint_names.size(); i++)
        {
            string joint_type;
            if(controller_name.find("gripper") != std::string::npos)
            {
                joint_type = "gripper_" + itr->second.joint_types[i];
            }
            else
            {
                joint_type = itr->second.joint_types[i];
            }
            mock->SetJointType(itr->second.joint_ids[i], joint_type);
        }
    }
    return true;
}

}
