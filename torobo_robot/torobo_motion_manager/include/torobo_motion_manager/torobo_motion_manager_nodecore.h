/**
 * @file  torobo_motion_manager_nodecore.h
 * @brief torobo motion_manager nodecore class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_MOTION_MANAGER_NODECORE_H
#define TOROBO_MOTION_MANAGER_NODECORE_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include "torobo_motion_manager/teaching_point_manager.h"
#include "torobo_motion_manager/teaching_trajectory_manager.h"
#include "torobo_motion_manager/move_home_position_action.h"
#include "torobo_motion_manager/move_teaching_point_action.h"
#include "torobo_motion_manager/move_teaching_trajectory_action.h"

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboMotionManagerNodeCore
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    std::vector<std::unique_ptr<TeachingPointManager>> tpm_;
    std::vector<std::unique_ptr<TeachingTrajectoryManager>> ttm_;
    std::vector<std::unique_ptr<MoveHomePositionActionServer>> move_home_as_;
    std::vector<std::unique_ptr<MoveTeachingPointActionServer>> move_tp_as_;
    std::vector<std::unique_ptr<MoveTeachingTrajectoryActionServer>> move_tt_as_;

    bool debug_;
    double rate_;
    double timeout_sec_;

public:
    ToroboMotionManagerNodeCore(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~ToroboMotionManagerNodeCore();

    void run();
    void runOnce();

    double getRate()
    {
        return rate_;
    }
};

}

#endif
