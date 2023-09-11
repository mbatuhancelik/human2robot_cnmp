/**
 * @file  torobo_joint_state_controller.h
 * @brief ToroboJointStateController class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_JOINT_STATE_CONTROLLER_H
#define TOROBO_JOINT_STATE_CONTROLLER_H


/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/callback_queue.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>


namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboJointStateController
{
public:
    ToroboJointStateController(ros::NodeHandle& node, std::string name);
    virtual ~ToroboJointStateController();
    void registerSourceTopic(std::string topic_name);
    void setPublishRate(double rate);
    void start();

protected:
    struct JointData
    {
        double position;
        double velocity;
        double effort;
        JointData() : position(0.0), velocity(0.0), effort(0.0) {}
        JointData(const JointData& org) {position = org.position; velocity = org.velocity; effort = org.effort;}
        JointData(double p, double v, double e) : position(p), velocity(v), effort(e) {}
    };
    void timerCallback(const ros::TimerEvent& e);
    void sourceCallback(const sensor_msgs::JointState::ConstPtr& msg);

private:
    // create child NodeHandle in order to separate parent node's thread
    std::unique_ptr<ros::NodeHandle> nh_ptr_;
    ros::CallbackQueue callback_queue_;
    std::unique_ptr<ros::NodeHandle> nh_timer_ptr_;
    ros::CallbackQueue callback_queue_timer_;
    std::unique_ptr<ros::AsyncSpinner> async_spinner_timer_ptr_;
 
    std::string name_;
    ros::Publisher pub_;
    ros::Timer timer_;
    std::map<std::string, JointData> jointdata_map_;
    std::vector<ros::Subscriber> sub_list_;
    double publish_rate_ = 50.0;
};

} // namespace torobo

#endif /* TOROBO_JOINT_STATE_CONTROLLER_H */

