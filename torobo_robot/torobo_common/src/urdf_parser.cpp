/**
 * @file  urdf_parser.cpp
 * @brief URDF parser class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include <iostream>
#include <torobo_common/urdf_parser.h>

using namespace std;

namespace torobo_common
{

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 Static Method Definitions
 ----------------------------------------------------------------------*/

urdf::ModelSharedPtr UrdfParser::getUrdf(const ros::NodeHandle& nh, const std::string& param_name)
{
    urdf::ModelSharedPtr urdf(new urdf::Model);

    std::string urdf_str;
    // Check for robot_description in proper namespace
    if (nh.getParam(param_name, urdf_str))
    {
        if (!urdf->initString(urdf_str))
        {
            ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter (namespace: " << nh.getNamespace() << ").");
            return urdf::ModelSharedPtr();
        }
    }
    // Check for robot_description in root
    else if (!urdf->initParam("robot_description"))
    {
        ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter");
        return urdf::ModelSharedPtr();
    }
    return urdf;
}

urdf::JointConstSharedPtr UrdfParser::getUrdfJoint(const urdf::Model& urdf, const std::string joint_name)
{
    urdf::JointConstSharedPtr urdf_joint = urdf.getJoint(joint_name);
    if (!urdf_joint)
    {
        ROS_ERROR_STREAM("Could not find joint '" << joint_name << "' in URDF model.");
        return nullptr;
    }
    return urdf_joint;
}

} // namespace torobo_common
