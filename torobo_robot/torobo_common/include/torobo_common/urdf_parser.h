/**
 * @file  urdf_parser.h
 * @brief URDF parser class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef URDF_PARSER_H
#define URDF_PARSER_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>
#include <urdf/model.h>

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
namespace torobo_common
{

class UrdfParser
{
public:
    static urdf::ModelSharedPtr getUrdf(const ros::NodeHandle& nh, const std::string& param_name="robot_description");
    static urdf::JointConstSharedPtr getUrdfJoint(const urdf::Model& urdf, const std::string joint_name);

protected:

private:
};

}

#endif
