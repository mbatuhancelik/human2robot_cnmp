/**
 * @file  torobo_joint_state_server.h
 * @brief ToroboJointStateServer class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_JOINT_STATE_SERVER_H
#define TOROBO_JOINT_STATE_SERVER_H


/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/callback_queue.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include "torobo_msgs/GetJointState.h"
#include "torobo_msgs/GetToroboJointState.h"
#include "torobo_msgs/ToroboJointState.h"


namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboJointStateServer
{
public:

    ToroboJointStateServer(ros::NodeHandle& node);
    virtual ~ToroboJointStateServer();

    void setPublishRate(double rate);
    void registerSourceTopics();
    void registerSourceTopic(std::string topic_name);
    void start();

protected:
    struct ControllerParam
    {
        std::string type;
        double publish_rate;
    };
    std::map<std::string, ControllerParam> getControllerParam();
    void timerCallback(const ros::TimerEvent& e);
    void sourceJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void sourceToroboJointStateCallback(const ros::MessageEvent<torobo_msgs::ToroboJointState const>& event);
    bool getJointStateService(torobo_msgs::GetJointState::Request &req, torobo_msgs::GetJointState::Response &res);
    bool getToroboJointStateService(torobo_msgs::GetToroboJointState::Request &req, torobo_msgs::GetToroboJointState::Response &res);

private:
    // child NodeHandle in order to separate callback thread
    std::unique_ptr<ros::NodeHandle> nh_ptr_;
    ros::CallbackQueue callback_queue_;
    std::unique_ptr<ros::NodeHandle> nh_timer_ptr_;
    ros::CallbackQueue callback_queue_timer_;
    std::unique_ptr<ros::AsyncSpinner> async_spinner_timer_ptr_;

    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::vector<ros::ServiceServer> services_;
    ros::Timer timer_;
    double publish_rate_ = 20;
    sensor_msgs::JointState::ConstPtr joint_state_ = nullptr; // Prohibit overwriting data pointed by the pointer.
    std::vector<ros::Subscriber> sub_list_;
    std::map<std::string, torobo_msgs::ToroboJointState::ConstPtr> torobojointstate_map_;
    std::map<std::string, ros::Publisher> pub_map_;
};

} // namespace torobo

#endif /* TOROBO_JOINT_STATE_SERVER_H */

