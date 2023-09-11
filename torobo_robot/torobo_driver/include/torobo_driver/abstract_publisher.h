/**
 * @file  abstract_publisher.h
 * @brief Abstract publisher class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef ABSTRACT_PUBLISHER_H
#define ABSTRACT_PUBLISHER_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include "MasterControllerClient/MasterControllerClient.h"

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class AbstractPublisher
{
public:
    AbstractPublisher(ros::NodeHandle &nh, const std::string& controller_name, const std::map<std::string, int>& jointsNameIdMap)
    : nh_(nh), controller_name_(controller_name), joints_name_id_map_(jointsNameIdMap) {}
    virtual ~AbstractPublisher() {pub_.shutdown();}
                                         
    virtual void Publish(const RecvPacket& packet, const ros::Time& rosTimeStamp) = 0;

protected:
    ros::NodeHandle& nh_;
    std::string controller_name_;
    std::map<std::string, int> joints_name_id_map_;
    ros::Publisher pub_;
};

}

#endif
