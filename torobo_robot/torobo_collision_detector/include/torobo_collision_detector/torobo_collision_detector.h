/**
 * @file  torobo_collision_detector.h
 * @brief Torobo cllision detector class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_COLLISION_DETECTOR_H
#define TOROBO_COLLISION_DETECTOR_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include "torobo_msgs/CheckCollision.h"
#include "torobo_msgs/GetCollisionInfo.h"
#include <sensor_msgs/JointState.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <unordered_map>

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboCollisionDetector
{
public:
    ToroboCollisionDetector(ros::NodeHandle& nh, const std::string& robot_description="robot_description",
        const std::string service_name_for_check_collision="check_collision", const std::string service_name_for_get_collision_info="get_collision_info",
        const double link_scale=1.0, const std::string link_name_regex=".*", const bool debug=false);
    virtual ~ToroboCollisionDetector();

protected:
    void InitializePlanningScene();
    void InitializeCollisionRobot(const double link_scale, const std::string link_name_regex);
    void InitializeJointStates();
    bool CheckCollisionService(torobo_msgs::CheckCollision::Request &req, torobo_msgs::CheckCollision::Response &res);
    bool GetCollisionInfoService(torobo_msgs::GetCollisionInfo::Request &req, torobo_msgs::GetCollisionInfo::Response &res);
    void SubsribeJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
    bool GetCollisionInfo(const sensor_msgs::JointState& req_joint_state, const collision_detection::CollisionRequest& creq, collision_detection::CollisionResult& cres);
    uint32_t ConvertBodyType(collision_detection::BodyType type);

    ros::NodeHandle& nh_;
    ros::ServiceServer service_for_check_collision_;
    ros::ServiceServer service_for_get_collision_info_;
    ros::Subscriber sub_;

    const std::string robot_description_param_name_;
    sensor_msgs::JointState joint_state_;
    std::unordered_map<std::string, int> joint_name_index_map_;

    double link_scale_;
    std::string link_name_regex_;
    bool debug_;

    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
};

}

#endif
