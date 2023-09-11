/**
 * @file  torobo_dynamics_nodecore.h
 * @brief torobo dynamics nodecore class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_DYNAMICS_NODECORE_H
#define TOROBO_DYNAMICS_NODECORE_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <memory.h>
#include <vector>
#include "torobo_dynamics/torobo_state.h"
#include "torobo_dynamics/torobo_dynamics.h"

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboDynamicsNodeCore
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::unique_ptr<torobo::ToroboState> torobo_state_;
    std::unique_ptr<torobo::ToroboDynamics> torobo_dyna_;

    bool debug_;
    bool sim_;
    double rate_;
    std::vector<std::string> controller_list_;
    bool is_init_;

public:
    ToroboDynamicsNodeCore(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~ToroboDynamicsNodeCore();

    void run();
    void runOnce();

    double getRate()
    {
        return rate_;
    }
};

}

#endif
