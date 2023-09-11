/**
 * @file  torobo_collision_detector_nodecore.h
 * @brief torobo collision detector nodecore class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef CURRENT_STATE_SELF_COLLISION_CHECKER_NODECORE_H
#define CURRENT_STATE_SELF_COLLISION_CHECKER_NODECORE_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <memory>
#include "torobo_collision_detector/current_state_self_collision_checker.h"

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class CurrentStateSelfCollisionCheckerNodeCore
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    bool is_init_ = false;
    bool sim_;
    double rate_;
    std::string service_name_for_check_collision_;

    std::unique_ptr<CurrentStateSelfCollisionChecker> checker_;

public:
    CurrentStateSelfCollisionCheckerNodeCore(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~CurrentStateSelfCollisionCheckerNodeCore();

    double getRate() { return rate_; }
    void run();
    void runOnce();
    bool isInit() { return is_init_; }
};

}

#endif
