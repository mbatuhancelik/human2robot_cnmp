/**
 * @file  torobo_collision_detector_nodecore.h
 * @brief torobo collision detector nodecore class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_COLLISION_DETECTOR_NODECORE_H
#define TOROBO_COLLISION_DETECTOR_NODECORE_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <memory>
#include "torobo_collision_detector/torobo_collision_detector.h"

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboCollisionDetectorNodeCore
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    bool debug_;
    std::string robot_description_name_;
    std::string service_name_for_check_collision_;
    std::string service_name_for_get_collision_info_;
    double link_scale_;
    std::string link_name_regex_;

    std::unique_ptr<ToroboCollisionDetector> detector_;

public:
    ToroboCollisionDetectorNodeCore(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~ToroboCollisionDetectorNodeCore();

    void run();
    void runOnce();
};

}

#endif
