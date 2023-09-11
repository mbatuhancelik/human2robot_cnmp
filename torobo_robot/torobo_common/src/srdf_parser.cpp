/**
 * @file  srdf_parser.cpp
 * @brief SRDF parser class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include <iostream>
#include <torobo_common/srdf_parser.h>
#include <tinyxml2.h>

using namespace std;

namespace torobo_common
{
/*----------------------------------------------------------------------
Forward Declarations
 ----------------------------------------------------------------------*/
static SrdfParser::GroupStateMap getGroupStateFromXmlDocument(tinyxml2::XMLDocument& doc);

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
SrdfParser::GroupStateMap SrdfParser::getGroupStateFromRosParam(ros::NodeHandle& nh,
    const std::string& srdf_param_name)
{
    string srdf_text;
    nh.param<string>(srdf_param_name, srdf_text, "");
    return getGroupStateFromText(srdf_text);
}

SrdfParser::GroupStateMap SrdfParser::getGroupStateFromFile(const std::string& srdf_file)
{
    tinyxml2::XMLDocument doc;
    doc.LoadFile(srdf_file.c_str());
    return getGroupStateFromXmlDocument(doc);
}

SrdfParser::GroupStateMap SrdfParser::getGroupStateFromText(const std::string& srdf_text)
{
    tinyxml2::XMLDocument doc;
    doc.Parse(srdf_text.c_str());
    return getGroupStateFromXmlDocument(doc);
}

/*----------------------------------------------------------------------
 Static Method Definitions
 ----------------------------------------------------------------------*/
static SrdfParser::GroupStateMap getGroupStateFromXmlDocument(tinyxml2::XMLDocument& doc)
{
    SrdfParser::GroupStateMap group_state_map;

    // tinyxml2::XMLElement * root = doc.RootElement();
    // tinyxml2::XMLElement * elem = doc.FirstChildElement("robot")->FirstChildElement("group_state");
    tinyxml2::XMLElement * elem = doc.FirstChildElement("robot");
    if(elem == NULL)
    {
        ROS_WARN("Not found robot tag in SRDF");
        return group_state_map;
    }
    elem = elem->FirstChildElement("group_state");
    if(elem == NULL)
    {
        ROS_WARN("Not found group_state tag in SRDF");
        return group_state_map;
    }

    while(elem)
    {
        string groupName = elem->Attribute("group");
        if(group_state_map.count(groupName) == 0)
        {
            group_state_map.insert(make_pair(groupName, map<string, SrdfParser::GroupState_t>()));
        }

        string groupStateName = elem->Attribute("name");

        SrdfParser::GroupState_t state;
        tinyxml2::XMLElement* joint = elem->FirstChildElement("joint");
        while(joint)
        {
            string jointName = joint->Attribute("name");
            string val = joint->Attribute("value");
            state.joint_names.push_back(jointName);
            state.positions.push_back(stod(val));
            joint = joint->NextSiblingElement("joint");
        }
        group_state_map[groupName].insert(make_pair(groupStateName, state));
        elem = elem->NextSiblingElement("group_state");
    }

    return group_state_map;
}

}
