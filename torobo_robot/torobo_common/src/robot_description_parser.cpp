/**
 * @file  robot_description_parser.cpp
 * @brief robot_description_parser class
 *
 * @par   Copyright © 2019 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "torobo_common/robot_description_parser.h"
#include <kdl_parser/kdl_parser.hpp>

using namespace std;

static const map<int, string> joint_type_string_map__ =
{
    {KDL::Joint::JointType::RotAxis,   "revolute"},
    {KDL::Joint::JointType::RotX,      "revolute"},
    {KDL::Joint::JointType::RotY,      "revolute"},
    {KDL::Joint::JointType::RotZ,      "revolute"},
    {KDL::Joint::JointType::TransAxis, "prismatic"},
    {KDL::Joint::JointType::TransX,    "prismatic"},
    {KDL::Joint::JointType::TransY,    "prismatic"},
    {KDL::Joint::JointType::TransZ,    "prismatic"},
    {KDL::Joint::JointType::None,      "fixed"},
};

namespace torobo_common
{
/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
bool parseRobotDescriptionFromRosParam(RobotDescription& description, const ros::NodeHandle& nh, const std::string& robot_description_param_name)
{
    //get robot_description parameter
    std::string robot_description_text;
    if(!nh.getParam(robot_description_param_name, robot_description_text))
    {
        ROS_ERROR("Failed to get rosparam [%s]", robot_description_param_name.c_str());
        return false;
    }

    return parseRobotDescriptionFromString(description, robot_description_text);
}

bool parseRobotDescriptionFromString(RobotDescription& description, const std::string& robot_description_text)
{
    description.segments.clear();

    KDL::Tree robot_tree;

    //parse urdf by using kdl_parser
    if(!kdl_parser::treeFromString(robot_description_text.c_str(), robot_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    std::map<std::string, KDL::TreeElement> robot_segment = robot_tree.getSegments();

    //access each data by using iterator
    for(auto itr = robot_segment.begin(); itr != robot_segment.end(); ++itr)
    {
        Segment segment;

        //get link name from std::map
        segment.link_name_ = itr->first;

        const KDL::Joint& joint = itr->second.segment.getJoint();
        segment.joint_name_ = joint.getName();
        if(joint_type_string_map__.count(joint.getType()) == 0)
        {
            segment.joint_type_ = "fixed";
        }
        else
        {
            segment.joint_type_ = joint_type_string_map__.at(joint.getType());
        }

        //get link parameters
        KDL::RigidBodyInertia rigid_body_inertia = itr->second.segment.getInertia();
        //get Mass of link
        segment.mass_ = rigid_body_inertia.getMass();
        //get Center of Gravity parameters
        KDL::Vector cog_point = rigid_body_inertia.getCOG();
        //get Center of Gravity position
        segment.cog_[0] = cog_point.x();
        segment.cog_[1] = cog_point.y();
        segment.cog_[2] = cog_point.z();

        description.segments.push_back(segment);
    }

    return true;
}

}