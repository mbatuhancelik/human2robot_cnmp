/**
 * @file  torobo_joint_state_publisher.h
 * @brief Torobo Joint state publisher class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_JOINT_STATE_PUBLISHER_H
#define TOROBO_JOINT_STATE_PUBLISHER_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include "torobo_driver/abstract_publisher.h"
#include "torobo_msgs/ToroboJointState.h"

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboJointStatePublisher : public AbstractPublisher
{
public:
    class Coef
    {
    public:
        Coef(const double pos=1.0,
             const double vel=1.0,
             const double acc=1.0,
             const double eft=1.0)
        {
            position = pos;
            velocity = vel;
            acceleration = acc;
            effort = eft;
        }
        ~Coef(){}

        double position;
        double velocity;
        double acceleration;
        double effort;
    };

    ToroboJointStatePublisher(ros::NodeHandle &nh,
                              const std::string& controller_name,
                              const std::map<std::string, int>& jointsNameIdMap,
                              const ToroboJointStatePublisher::Coef& coef=Coef(),
                              const int queueSize=1);
    virtual ~ToroboJointStatePublisher();

    void Publish(const RecvPacket& packet, const ros::Time& rosTimeStamp) override;

protected:
    void Initialize(const int queueSize);

    ToroboJointStatePublisher::Coef coef_;
    torobo_msgs::ToroboJointState torobo_joint_state_;
};

}

#endif
