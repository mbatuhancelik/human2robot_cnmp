/**
 * @file  teaching_trajectory_manager.h
 * @brief Teaching trajectory manager class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TEACHING_TRAJECTORY_MANAGER_H
#define TEACHING_TRAJECTORY_MANAGER_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <vector>
#include <thread>
#include "torobo_msgs/DeleteTeachingTrajectory.h"
#include "torobo_msgs/GetTeachingTrajectory.h"
#include "torobo_msgs/GetTeachingTrajectoryNames.h"
#include "torobo_msgs/RecordTeachingTrajectory.h"

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
namespace torobo
{

class TeachingTrajectoryManager
{
public:
    TeachingTrajectoryManager(ros::NodeHandle& nh,
        const std::string& controller_name,
        const std::vector<std::string>& joint_names,
        const double rate
    );
    virtual ~TeachingTrajectoryManager();

protected:
    bool DeleteTeachingTrajectoryService(torobo_msgs::DeleteTeachingTrajectory::Request &req, torobo_msgs::DeleteTeachingTrajectory::Response &res);
    bool GetTeachingTrajectoryService(torobo_msgs::GetTeachingTrajectory::Request &req, torobo_msgs::GetTeachingTrajectory::Response &res);
    bool GetTeachingTrajectoryNamesService(torobo_msgs::GetTeachingTrajectoryNames::Request &req, torobo_msgs::GetTeachingTrajectoryNames::Response &res);
    bool RecordTeachingTrajectoryService(torobo_msgs::RecordTeachingTrajectory::Request &req, torobo_msgs::RecordTeachingTrajectory::Response &res);

private:
    void queueThread();
    std::vector<std::string> GetTeachingTrajectoryNames();

    std::unique_ptr<std::thread> thread_;
    bool thread_working_;
    double duration_;
    
    ros::NodeHandle nh_;
    ros::CallbackQueue cb_queue_;
    std::vector<ros::AdvertiseServiceOptions> service_options_;
    std::vector<ros::ServiceServer> services_;
    const std::string controller_name_;
    const std::vector<std::string> joint_names_;
};

}

#endif
