/**
 * @file  srdf_parser.h
 * @brief SRDF parser class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef SRDF_PARSER_H
#define SRDF_PARSER_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
namespace torobo_common
{

class SrdfParser
{
public:
    typedef struct
    {
        std::vector<std::string> joint_names;
        std::vector<double> positions;
    }GroupState_t;
    typedef std::map<std::string, std::map<std::string, SrdfParser::GroupState_t>> GroupStateMap;

    static SrdfParser::GroupStateMap getGroupStateFromRosParam(ros::NodeHandle& nh,
        const std::string& srdf_param_name="robot_description_semantic");
    static SrdfParser::GroupStateMap getGroupStateFromFile(const std::string& srdf_file);
    static SrdfParser::GroupStateMap getGroupStateFromText(const std::string& srdf_text);

protected:

private:
};

}

#endif
