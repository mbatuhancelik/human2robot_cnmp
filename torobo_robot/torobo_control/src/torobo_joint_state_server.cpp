/**
 * @file  torobo_joint_state_server.cpp
 * @brief ToroboJointStateServer class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include "torobo_control/torobo_joint_state_server.h"


using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Public Method Implementations
 ----------------------------------------------------------------------*/
ToroboJointStateServer::ToroboJointStateServer(ros::NodeHandle& node)
{
    nh_ptr_.reset(new ros::NodeHandle(node));
    nh_ptr_->setCallbackQueue(&callback_queue_);
    nh_timer_ptr_.reset(new ros::NodeHandle(node));
    nh_timer_ptr_->setCallbackQueue(&callback_queue_timer_);
    async_spinner_timer_ptr_.reset(new ros::AsyncSpinner(1, &callback_queue_timer_));
    async_spinner_timer_ptr_->start();

    sub_ = nh_ptr_->subscribe("joint_states", 1, &ToroboJointStateServer::sourceJointStateCallback, this, ros::TransportHints().reliable().tcpNoDelay(true));
    pub_ = nh_ptr_->advertise<sensor_msgs::JointState>("joint_state_server/joint_states", 1, this);
    services_.push_back(nh_ptr_->advertiseService("joint_state_server/get_joint_state", &ToroboJointStateServer::getJointStateService, this));
    services_.push_back(nh_ptr_->advertiseService("joint_state_server/get_torobo_joint_state", &ToroboJointStateServer::getToroboJointStateService, this));
}

ToroboJointStateServer::~ToroboJointStateServer()
{
    timer_.stop(); // this stop is needed because ros::Timer uses global callback_queue different with custom queue.
    sub_.shutdown();
    pub_.shutdown();
    for(auto itr = services_.begin(); itr != services_.end(); ++itr)
    {
        itr->shutdown();
    }
    async_spinner_timer_ptr_->stop();  // wait for finishing async spinner's thread
}

void ToroboJointStateServer::setPublishRate(double rate)
{
    publish_rate_ = rate;
}

void ToroboJointStateServer::registerSourceTopics()
{
    std::map<std::string, ControllerParam> controller_param = getControllerParam();

    for (const auto itr : controller_param)
    {
        auto name = itr.first;
        auto param = itr.second;

        if(param.type.find("JointTrajectoryController") != std::string::npos)
        {
            std::string topic_name = name + "/torobo_joint_state";
            ROS_INFO_STREAM("[torobo_joint_state_server] register source topic: " << topic_name);
            registerSourceTopic(topic_name);
        }

        else if(param.type.find("GripperActionController") != std::string::npos)
        {
            std::string topic_name = name + "/torobo_joint_state";
            ROS_INFO_STREAM("[torobo_joint_state_server] register source topic: " << topic_name);
            registerSourceTopic(topic_name);
        }
    }
}

void ToroboJointStateServer::registerSourceTopic(std::string topic_name)
{
    sub_list_.push_back(nh_ptr_->subscribe(topic_name, 1, &ToroboJointStateServer::sourceToroboJointStateCallback, this, ros::TransportHints().reliable().tcpNoDelay(true)) );
    torobojointstate_map_[nh_ptr_->resolveName(topic_name)] = NULL;
    pub_map_[nh_ptr_->resolveName(topic_name)] = nh_ptr_->advertise<torobo_msgs::ToroboJointState>("joint_state_server/" + topic_name, 1, this);
}

void ToroboJointStateServer::start()
{
    timer_ = nh_timer_ptr_->createTimer(ros::Duration(1.0 / (double)publish_rate_), &ToroboJointStateServer::timerCallback, this);
    timer_.start();
}

/*----------------------------------------------------------------------
 Protected Method Implementations
 ----------------------------------------------------------------------*/
std::map<std::string, ToroboJointStateServer::ControllerParam> ToroboJointStateServer::getControllerParam()
{
    std::vector<std::string> controller_list;
    nh_ptr_->getParam("controller_list", controller_list);

    std::map<std::string, ToroboJointStateServer::ControllerParam> controller_param;

    for (auto name: controller_list)
    {
        ToroboJointStateServer::ControllerParam param;
        nh_ptr_->param<std::string>(name + "/type", param.type, "");
        nh_ptr_->param<double>(name + "/rate", param.publish_rate, 20.0);
        controller_param[name] = param;
    }

    return controller_param;    
}

void ToroboJointStateServer::sourceJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_state_ = msg;
}

void ToroboJointStateServer::sourceToroboJointStateCallback(const ros::MessageEvent<torobo_msgs::ToroboJointState const>& event)
{
    const ros::M_string& header = event.getConnectionHeader();
    std::string topic = header.at("topic");
    const torobo_msgs::ToroboJointState::ConstPtr& msg = event.getMessage();
    torobojointstate_map_[topic] = msg;
}

void ToroboJointStateServer::timerCallback(const ros::TimerEvent& e)
{
    callback_queue_.callAvailable();

    if(!joint_state_)
    {
        // joint_state message seems to be not ready.
        return;
    }

    // publish JointState
    pub_.publish(joint_state_);

    // publish ToroboJointState
    for(auto itr = pub_map_.begin(); itr != pub_map_.end(); ++itr)
    {
        const torobo_msgs::ToroboJointState::ConstPtr& msg = torobojointstate_map_[itr->first];
        if(msg)
        {
            itr->second.publish(msg);
        }
    }
}

bool ToroboJointStateServer::getJointStateService(torobo_msgs::GetJointState::Request &req, torobo_msgs::GetJointState::Response &res)
{
    if(joint_state_)
    {
        res.jointState = *joint_state_;
    }
    else
    {
        ROS_ERROR("joint_state data has not ready yet.");
        return false;
    }

    return true;
}

bool ToroboJointStateServer::getToroboJointStateService(torobo_msgs::GetToroboJointState::Request &req, torobo_msgs::GetToroboJointState::Response &res)
{
    const string& controllerName = req.controllerName;
    for(auto itr = pub_map_.begin(); itr != pub_map_.end(); ++itr)
    {
        if (itr->first.find(controllerName) == std::string::npos)
        {
            continue;
        }
        if(!torobojointstate_map_[itr->first])
        {
            // ROS_ERROR("torobo joint_state data has not configured yet.");
            return false;
        }
        res.toroboJointState = *torobojointstate_map_[itr->first];
        return true;
    }
    ROS_ERROR("given controller name is not found.");
    return false;
}

} // namespace torobo
