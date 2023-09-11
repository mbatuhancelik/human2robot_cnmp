/**
 * @file  torobo_joint_state_controller.cpp
 * @brief ToroboJointStateController class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include "torobo_control/torobo_joint_state_controller.h"


using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Public Method Implementations
 ----------------------------------------------------------------------*/
ToroboJointStateController::ToroboJointStateController(ros::NodeHandle& node, std::string name) : name_(name)
{
    nh_ptr_.reset(new ros::NodeHandle(node));
    nh_ptr_->setCallbackQueue(&callback_queue_);
    nh_timer_ptr_.reset(new ros::NodeHandle(node));
    nh_timer_ptr_->setCallbackQueue(&callback_queue_timer_);
    async_spinner_timer_ptr_.reset(new ros::AsyncSpinner(1, &callback_queue_timer_));
    async_spinner_timer_ptr_->start();

    pub_ = nh_ptr_->advertise<sensor_msgs::JointState>("joint_states", 1, this);
}

ToroboJointStateController::~ToroboJointStateController()
{
    timer_.stop(); // this stop is needed because ros::Timer uses global callback_queue different with custom queue.
    for (auto itr = sub_list_.begin(); itr != sub_list_.end(); ++itr)
    {
        itr->shutdown();
    }
    pub_.shutdown();
    async_spinner_timer_ptr_->stop();  // wait for finishing async spinner's thread
}

void ToroboJointStateController::registerSourceTopic(std::string topic_name)
{
    sub_list_.push_back( nh_ptr_->subscribe(
        topic_name, 1, &ToroboJointStateController::sourceCallback, this, ros::TransportHints().reliable().tcpNoDelay(true)
    ));
}

void ToroboJointStateController::setPublishRate(double rate)
{
    publish_rate_ = rate;
}

void ToroboJointStateController::start()
{
    timer_ = nh_timer_ptr_->createTimer(ros::Duration(1.0 / (double)publish_rate_), &ToroboJointStateController::timerCallback, this);
}

/*----------------------------------------------------------------------
 Protected Method Implementations
 ----------------------------------------------------------------------*/
void ToroboJointStateController::sourceCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for(int i=0; i<msg->name.size(); i++)
    {
        jointdata_map_[msg->name[i]] = JointData(msg->position[i], msg->velocity[i], msg->effort[i]);
    }
}

void ToroboJointStateController::timerCallback(const ros::TimerEvent& e)
{
    callback_queue_.callAvailable();

    sensor_msgs::JointState::Ptr msg(new sensor_msgs::JointState());

    msg->header.stamp = ros::Time::now();
    for(auto itr = jointdata_map_.begin(); itr != jointdata_map_.end(); ++itr)
    {
        msg->name.push_back( itr->first );
        msg->position.push_back( itr->second.position );
        msg->velocity.push_back( itr->second.velocity );
        msg->effort.push_back( itr->second.effort );
    }

    pub_.publish(msg);
}

} // namespace torobo
