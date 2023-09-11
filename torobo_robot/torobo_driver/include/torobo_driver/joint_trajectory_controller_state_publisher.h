/**
 * @file  joint_trajectory_controller_state_publisher.h
 * @brief Joint trajectory controller state publisher class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef JOINT_TRAJECTORY_CONTROLLER_STATE_PUBLISHER_H
#define JOINT_TRAJECTORY_CONTROLLER_STATE_PUBLISHER_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include "torobo_driver/abstract_publisher.h"
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class JointTrajectoryControllerStatePublisher : public AbstractPublisher
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

    JointTrajectoryControllerStatePublisher(ros::NodeHandle &nh,
                                            const std::string& controller_name,
                                            const std::map<std::string, int>& jointsNameIdMap,
                                            const JointTrajectoryControllerStatePublisher::Coef& coef=Coef(),
                                            const int queueSize=1);
    virtual ~JointTrajectoryControllerStatePublisher();

    void Publish(const RecvPacket& packet, const ros::Time& rosTimeStamp) override;

protected:
    void Initialize(const int queueSize);

    JointTrajectoryControllerStatePublisher::Coef coef_;
    control_msgs::JointTrajectoryControllerState joint_trajectory_controller_state_;
};

}

#endif
