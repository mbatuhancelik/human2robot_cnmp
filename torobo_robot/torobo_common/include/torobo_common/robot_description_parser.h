/**
 * @file  robot_description_parser.h
 * @brief robot_description_parser class
 *
 * @par   Copyright © 2019 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef ROBOT_DESCRIPTION_PARSER_H
#define ROBOT_DESCRIPTION_PARSER_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <vector>
#include <map>
#include <string>
#include <ros/ros.h>

namespace torobo_common
{

class Segment
{
public:
    Segment() :
        link_name_(""),
        joint_name_(""),
        joint_type_(""),
        mass_(0.0),
        cog_{0}
    {
    }

    ~Segment()
    {
    }

    std::string link_name_;
    std::string joint_name_;
    std::string joint_type_;
    double mass_;
    double cog_[3];
};

class RobotDescription
{
public:
    RobotDescription()
    {
    }

    ~RobotDescription()
    {
    }

    std::vector<Segment> segments;

};

/*----------------------------------------------------------------------
 Public Method Declarations
 ----------------------------------------------------------------------*/
bool parseRobotDescriptionFromRosParam(RobotDescription& description, const ros::NodeHandle& nh, const std::string& robot_description_param_name="robot_description");
bool parseRobotDescriptionFromString(RobotDescription& description, const std::string& robot_description_text);

}

#endif
