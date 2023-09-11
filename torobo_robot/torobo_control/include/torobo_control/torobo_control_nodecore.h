/**
 * @file  torobo_control_nodecore.h
 * @brief ToroboControlNodeCore class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_CONTROL_NODECORE_H
#define TOROBO_CONTROL_NODECORE_H


/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include "torobo_control/torobo_controller_spawner.h"
#include "torobo_control/torobo_joint_state_server.h"


namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboControlNodeCore
{
public:
    ToroboControlNodeCore(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~ToroboControlNodeCore();
    void run();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::unique_ptr<ToroboControllerSpawner> controller_spawner_;
    std::unique_ptr<ToroboJointStateServer> joint_state_server_;
};

} // namespace torobo

#endif /* TOROBO_CONTROL_NODECORE_H */
