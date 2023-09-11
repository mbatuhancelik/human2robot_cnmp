/**
 * @file  current_state_self_collision_checker.h
 * @brief current_state_self_collision_checker class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef CURRENT_STATE_SELF_COLLISION_CHECKER_H
#define CURRENT_STATE_SELF_COLLISION_CHECKER_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <map>
#include <vector>
#include "torobo_msgs/CheckCollision.h"
#include "torobo_msgs/ServoOff.h"
#include "torobo_msgs/GetServoState.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class CurrentStateSelfCollisionChecker
{
private:
    ros::NodeHandle nh_;
    bool sim_;
    bool is_init_ = false;

    // diagnostics for command collision
    diagnostic_updater::Updater diag_updater_;
    bool is_self_colliding_ = false;
    bool entering_self_collision_state_ = false;

    ros::ServiceClient check_collision_srv_;
    std::map<std::string, ros::ServiceClient> servo_off_srvs_;
    std::map<std::string, ros::ServiceClient> get_servo_state_srvs_;
    std::map<std::string, std::vector<std::string>> controller_joints_map_;

    bool isSelfColliding();
    bool isServoOnAnyJoint(const std::string& controller_name, const std::vector<std::string>& joint_names);
    void servoOff(const std::string& controller_name, const std::vector<std::string>& joint_names);
    void executeServoOff();
    void diagnosticCheckSelfCollision(diagnostic_updater::DiagnosticStatusWrapper &stat);

public:
    CurrentStateSelfCollisionChecker(
        const ros::NodeHandle& nh,
        const std::map<std::string, std::vector<std::string>>& controller_joints_map,
        bool sim,
        std::string service_name_for_check_collision = "check_collision"
    );
    ~CurrentStateSelfCollisionChecker();

    void checkSelfCollision();
    bool isInit() { return is_init_; }

};

}

#endif
