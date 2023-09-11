/**
 * @file  rosparam_loader.h
 * @brief rosparam_loader class
 *
 * @par   Copyright © 2019 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef ROSPARAM_LOADER_H
#define ROSPARAM_LOADER_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <vector>
#include <map>
#include <string>
#include <ros/ros.h>

namespace torobo_common
{

/*----------------------------------------------------------------------
 Public Method Declarations
 ----------------------------------------------------------------------*/
bool getControllerJointsMap( std::map<std::string, std::vector<std::string>>& controller_joints_map, const ros::NodeHandle& nh);

}

#endif
