/**
 * @file  joint_state_publisher.h
 * @brief Joint state publisher class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef JOINT_STATE_PUBLISHER_H
#define JOINT_STATE_PUBLISHER_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include "torobo_driver/abstract_publisher.h"
#include <sensor_msgs/JointState.h>

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class JointStatePublisher : public AbstractPublisher
{
public:
    class Coef
    {
    public:
        Coef(const double pos=1.0,
             const double vel=1.0,
             const double eft=1.0)
        {
            position = pos;
            velocity = vel;
            effort = eft;
        }
        ~Coef(){}

        double position;
        double velocity;
        double effort;
    };

    JointStatePublisher(ros::NodeHandle& nh,
                        const std::string& controller_name,
                        const std::map<std::string, int>& jointsNameIdMap,
                        const JointStatePublisher::Coef& coef=Coef(),
                        const int queueSize=1);
    virtual ~JointStatePublisher();

    void Publish(const RecvPacket& packet, const ros::Time& rosTimeStamp) override;

protected:
    void Initialize(const int queueSize);

    JointStatePublisher::Coef coef_;
    sensor_msgs::JointState joint_state_;
};

}

#endif
