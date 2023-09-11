/**
 * @file  torobo_collision_detector.cpp
 * @brief Torobo cllision detector class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "torobo_collision_detector/torobo_collision_detector.h"
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <torobo_common/math_util.h>
#include <chrono>
#include <regex>

using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
ToroboCollisionDetector::ToroboCollisionDetector(ros::NodeHandle& nh, const std::string& robot_description, const std::string service_name_for_check_collision,
     const std::string service_name_for_get_collision_info, const double link_scale, const std::string link_name_regex, const bool debug)
     : nh_(nh), robot_description_param_name_(robot_description), link_scale_(link_scale), link_name_regex_(link_name_regex), debug_(debug)
{
    InitializePlanningScene();
    InitializeCollisionRobot(link_scale, link_name_regex);
    InitializeJointStates();
    service_for_check_collision_ = nh_.advertiseService(service_name_for_check_collision, &ToroboCollisionDetector::CheckCollisionService, this);
    service_for_get_collision_info_ = nh_.advertiseService(service_name_for_get_collision_info, &ToroboCollisionDetector::GetCollisionInfoService, this);
    sub_ = nh_.subscribe("joint_states", 1, &ToroboCollisionDetector::SubsribeJointStatesCallback, this, ros::TransportHints().reliable().tcpNoDelay(true));
}

ToroboCollisionDetector::~ToroboCollisionDetector()
{
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
void ToroboCollisionDetector::InitializePlanningScene()
{
  	psm_ = planning_scene_monitor::PlanningSceneMonitorPtr(new planning_scene_monitor::PlanningSceneMonitor(robot_description_param_name_));
}

void ToroboCollisionDetector::InitializeCollisionRobot(const double link_scale, const std::string link_name_regex)
{
    if(math::nearly_equal(link_scale, 1.0))
    {
        return;
    }

    planning_scene_monitor::LockedPlanningSceneRW lps(psm_);

    std::regex query(link_name_regex);

    // set LinkScale at CollisionRobot at initialize in order to reduce computational cost.
    if(lps)
    {
        collision_detection::CollisionRobotPtr crobot = lps->getCollisionRobotNonConst();
#if 0 // this block changes all link's scale.
        crobot->setScale(link_scale);
#else // this block changes scale of links specified by regex.
        auto scale_map = crobot->getLinkScale();
        for(auto itr = scale_map.begin(); itr != scale_map.end(); ++itr)
        {
            std::string link_name = itr->first;
            if(std::regex_match(link_name, query))
            {
                itr->second = link_scale;
                ROS_INFO_STREAM("[torobo_collision_detector] CollisionRobot link_scale of [" << itr->first << "] is changed to " << link_scale);
            }
        }
        crobot->setLinkScale(scale_map);
#endif
        lps->propogateRobotPadding();
    }
}

void ToroboCollisionDetector::InitializeJointStates()
{
    planning_scene_monitor::LockedPlanningSceneRW lps(psm_);

    if(lps)
    {
        robot_state::RobotState& state = lps->getCurrentStateNonConst();
        vector<string> jointNames = lps->getCurrentState().getVariableNames();
        for(auto itr = jointNames.begin(); itr != jointNames.end(); ++itr)
        {
            joint_state_.name.push_back(*itr);
            joint_state_.position.push_back(0.0);
            joint_state_.velocity.push_back(0.0);
            joint_state_.effort.push_back(0.0);
            joint_name_index_map_.insert(make_pair(*itr, (int)joint_name_index_map_.size()));
        }
    }
}

bool ToroboCollisionDetector::GetCollisionInfo(const sensor_msgs::JointState& req_joint_state, const collision_detection::CollisionRequest& creq, collision_detection::CollisionResult& cres)
{
    std::chrono::system_clock::time_point start;
    if (debug_)
    {
        start = std::chrono::system_clock::now();
    }

    planning_scene_monitor::LockedPlanningSceneRW lps(psm_);

    if(!lps)
    {
        ROS_ERROR("Planning scene not configured");
        return false;
    }
    else
    {
        ////////////////////////////////
        // prepare current RobotState //
        ////////////////////////////////
        robot_state::RobotState& state = lps->getCurrentStateNonConst();
        sensor_msgs::JointState checkJointState = joint_state_;

        const size_t size = req_joint_state.name.size();
        for(size_t i = 0; i < req_joint_state.name.size(); i++)
        {
            if(joint_name_index_map_.count(req_joint_state.name[i]) > 0)
            {
                const int idx = joint_name_index_map_[req_joint_state.name[i]];
                checkJointState.position[idx] = req_joint_state.position[i];
            }
        }

        for(size_t i = 0; i < checkJointState.name.size(); i++)
        {
            state.setJointPositions(checkJointState.name[i], &checkJointState.position[i]);
        }

        if(math::nearly_equal(link_scale_, 1.0))
        {
            // check collision by means of planning scene
            lps->checkSelfCollision(creq, cres);
        }
        else
        {
            // check collision by means of CollisionRobot (exists inside of planning scene)
            //    this process costs time a bit more than planning scene's checkSelfCollision
            //    e.g. it costs 0.5 msec(when using planning_scene.checkSelfCollision) --> 1.5 msec(when using collision_robot.checkSelfCollision) in 3.5GHz CPU.
            // When we use CollisionRobot, both planning scene and allowed_collision_matrix must be got and set into itself
            //    (otherwise, warning of 'dirty collision' will be reported).
            collision_detection::CollisionRobotPtr crobot = lps->getCollisionRobotNonConst();
            moveit_msgs::PlanningScene scene;
            lps->getPlanningSceneMsg(scene);
            lps->setPlanningSceneMsg(scene);  // aplly scene's link_scale to collision robot
            crobot->checkSelfCollision(creq, cres, state, scene.allowed_collision_matrix); // apply scene's allowed_collision_matrix. (otherwise, all collision will be detected.)
        }
    }

    if(debug_)
    {
        auto end = std::chrono::system_clock::now();
        auto dur = end - start;
        auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(dur).count();
        ROS_INFO_STREAM("cost time of checking collision  = " << nsec << " [ns]");
    }

    return true;
}


bool ToroboCollisionDetector::CheckCollisionService(torobo_msgs::CheckCollision::Request &req, torobo_msgs::CheckCollision::Response &res)
{
    // prepare request msg
    collision_detection::CollisionRequest creq;
    collision_detection::CollisionResult cres;
    creq.contacts = false;
    creq.max_contacts = 0;

    // call function
    if( !GetCollisionInfo(req.jointState, creq, cres) )
    {
        return false;
    }

    // set result
    res.isColliding = cres.collision;

    return true;
}

bool ToroboCollisionDetector::GetCollisionInfoService(torobo_msgs::GetCollisionInfo::Request &req, torobo_msgs::GetCollisionInfo::Response &res)
{
    // prepare request msg
    collision_detection::CollisionRequest creq;
    collision_detection::CollisionResult cres;
    creq.max_contacts = req.collision_info_request.max_contacts;
    creq.contacts = (creq.max_contacts > 0) ? true : false;

    // call function
    if( !GetCollisionInfo(req.collision_info_request.jointState, creq, cres) )
    {
        return false;
    }

    // set result
    res.collision_info_response.isColliding = cres.collision;
    if(cres.collision && creq.max_contacts > 0)
    {
        for(const auto& x : cres.contacts)
        {
            moveit_msgs::ContactInformation contact;
            contact.contact_body_1 = x.second[0].body_name_1;
            contact.body_type_1 = ConvertBodyType(x.second[0].body_type_1);
            contact.contact_body_2 = x.second[0].body_name_2;
            contact.body_type_2 = ConvertBodyType(x.second[0].body_type_2);
            res.collision_info_response.contacts.push_back(contact);
        }
    }

    return true;
}

uint32_t ToroboCollisionDetector::ConvertBodyType(collision_detection::BodyType type)
{
    switch(type)
    {
    case collision_detection::BodyType::ROBOT_LINK:
        return moveit_msgs::ContactInformation::ROBOT_LINK;
    case collision_detection::BodyType::WORLD_OBJECT:
        return moveit_msgs::ContactInformation::WORLD_OBJECT;
    case collision_detection::BodyType::ROBOT_ATTACHED:
        return moveit_msgs::ContactInformation::ROBOT_ATTACHED;
    default:
        return moveit_msgs::ContactInformation::ROBOT_LINK;
    }
}

void ToroboCollisionDetector::SubsribeJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for(size_t i = 0; i < msg->name.size(); i++)
    {
        if(i >= msg->position.size())
        {
            break;
        }

        const string& name = msg->name[i];
        if(joint_name_index_map_.count(name) > 0)
        {
            const int idx = joint_name_index_map_[name];
            joint_state_.position[idx] = msg->position[i];
        }
    }
}

}
