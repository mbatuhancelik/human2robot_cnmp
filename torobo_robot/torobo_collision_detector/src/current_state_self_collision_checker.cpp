/**
 * @file  current_state_self_collision_checker.cpp
 * @brief current_state_self_collision_checker class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */
#include "torobo_collision_detector/current_state_self_collision_checker.h"

#include <torobo_common/ros_util.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <std_msgs/Bool.h>

using namespace std;

namespace torobo
{

CurrentStateSelfCollisionChecker::CurrentStateSelfCollisionChecker(
    const ros::NodeHandle &nh,
    const std::map<std::string, std::vector<std::string>> &controller_joints_map,
    bool sim,
    std::string service_name_for_check_collision)
    : nh_(nh), controller_joints_map_(controller_joints_map), sim_(sim), is_init_(false)
{
    const int max_retry_num = 10;
    const ros::Duration wait_for_service_timeout(5.0);

    if (!torobo_common::waitForServiceWithRetry<torobo_msgs::CheckCollision>(
            check_collision_srv_,
            nh_, service_name_for_check_collision, max_retry_num, wait_for_service_timeout))
    {
        ROS_FATAL("Failed to create CheckCollisionServiceClient.");
        return;
    }

    // diagnostics
    diag_updater_.setHardwareID("Collision");
    diag_updater_.add("Check Self Collision", this, &CurrentStateSelfCollisionChecker::diagnosticCheckSelfCollision);

    if (sim_)
    {
        is_init_ = true;
        return;
    }

    // create servo_off & get_servo_state service clients when real hardware mode
    for (const auto &itr : controller_joints_map_)
    {
        const string &controller_name = itr.first;

        ros::ServiceClient servo_off_srv;
        if (!torobo_common::waitForServiceWithRetry<torobo_msgs::ServoOff>(
                servo_off_srv,
                nh_, controller_name + "/servo_off", max_retry_num, wait_for_service_timeout))
        {
            ROS_FATAL("Failed to create ServoOffServiceClient.");
            return;
        }
        servo_off_srvs_.insert(make_pair(controller_name, servo_off_srv));

        ros::ServiceClient get_servo_state_srv;
        if (!torobo_common::waitForServiceWithRetry<torobo_msgs::GetServoState>(
                get_servo_state_srv,
                nh_, controller_name + "/get_servo_state", max_retry_num, wait_for_service_timeout))
        {
            ROS_FATAL("Failed to create GetServoStatusServiceClient.");
            return;
        }
        get_servo_state_srvs_.insert(make_pair(controller_name, get_servo_state_srv));
    }

    if (servo_off_srvs_.size() != controller_joints_map_.size() || get_servo_state_srvs_.size() != controller_joints_map_.size())
    {
        string msg = "current state self collision checker did not found all controller services.";
        msg += "\nThe self-collision detection result may not work well. ";
        ROS_FATAL("%s", msg.c_str());
        return;
    }

    is_init_ = true;
}

CurrentStateSelfCollisionChecker::~CurrentStateSelfCollisionChecker()
{
}

bool CurrentStateSelfCollisionChecker::isSelfColliding()
{
    if (!check_collision_srv_.waitForExistence(ros::Duration(0.1)))
    {
        ROS_WARN("check_collision_service is not exists.");
        return false;
    }

    torobo_msgs::CheckCollision::Request req;
    torobo_msgs::CheckCollision::Response res;
    if (!check_collision_srv_.call(req, res))
    {
        ROS_WARN("Failed to call check_collision service.");
        return false;
    }

    return res.isColliding;
}

bool CurrentStateSelfCollisionChecker::isServoOnAnyJoint(const std::string &controller_name, const std::vector<std::string> &joint_names)
{
    auto itr = get_servo_state_srvs_.find(controller_name);
    if (itr == get_servo_state_srvs_.end())
    {
        ROS_WARN("[%s] get_servo_state service did not found in internal map.", controller_name.c_str());
        return false;
    }
    ros::ServiceClient &get_servo_state_srv = itr->second;

    if (!get_servo_state_srv.waitForExistence(ros::Duration(0.1)))
    {
        ROS_WARN("[%s] get_servo_state service is not exists.", controller_name.c_str());
        return false;
    }

    torobo_msgs::GetServoState::Request req;
    torobo_msgs::GetServoState::Response res;
    req.joint_names = move(joint_names);
    if (!get_servo_state_srv.call(req, res))
    {
        ROS_WARN("[%s] failed to call get_servo_state service.", controller_name.c_str());
        return false;
    }

    // check servo on/off
    for (const auto &is_servo_on : res.is_servo_on)
    {
        if (is_servo_on)
        {
            return true;
        }
    }
    return false;
}

void CurrentStateSelfCollisionChecker::servoOff(const std::string &controller_name, const std::vector<std::string> &joint_names)
{
    auto itr = servo_off_srvs_.find(controller_name);
    if (itr == servo_off_srvs_.end())
    {
        ROS_WARN("[%s] servo_off service did not found in internal map.", controller_name.c_str());
        return;
    }

    ros::ServiceClient &servo_off_srv = itr->second;
    if (!servo_off_srv.waitForExistence(ros::Duration(0.1)))
    {
        ROS_WARN("[%s] servo_off service is not exists.", controller_name.c_str());
        return;
    }

    torobo_msgs::ServoOff::Request req;
    torobo_msgs::ServoOff::Response res;
    req.joint_names = move(joint_names);
    if (!servo_off_srv.call(req, res))
    {
        ROS_WARN("[%s] failed to call servo_off service.", controller_name.c_str());
        return;
    }
}

void CurrentStateSelfCollisionChecker::executeServoOff()
{
    for (const auto &itr : controller_joints_map_)
    {
        const string &controller_name = itr.first;
        const vector<string> &joint_names = itr.second;

        // Publish servo off command only to a controller having a joint in the servo on state.
        // [detail]
        // If the servo OFF command is always published, the brake release for self-collision
        // return can not be performed (the brake is turned ON by the servo OFF command).
        if (isServoOnAnyJoint(controller_name, joint_names))
        {
            ROS_WARN("servo off [%s] all joints.", controller_name.c_str());
            servoOff(controller_name, joint_names);
        }
    }
}

void CurrentStateSelfCollisionChecker::diagnosticCheckSelfCollision(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    if (is_self_colliding_)
    {
        if (!entering_self_collision_state_)
        {
            entering_self_collision_state_ = true;
            ROS_WARN("Entering self collision state!");
        }
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "ERROR: Self-Collision has occurred");
    }
    else
    {
        if (entering_self_collision_state_)
        {
            ROS_INFO("Recovered from self collision state.");
            entering_self_collision_state_ = false;
        }
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    }
}

void CurrentStateSelfCollisionChecker::checkSelfCollision()
{
    if(!is_init_)
    {
        ROS_ERROR("[CurrentStateSelfCollisionChecker] is not initialized!");
        return;
    }

    // check isSelfColliding()
    is_self_colliding_ = isSelfColliding();

    // update diagnostic
    diag_updater_.force_update();

    // do emergency servo off when isSelfColliding is True
    if (is_self_colliding_)
    {
        if (!sim_)
        {
            executeServoOff();
        }
    }
}

} // namespace torobo
