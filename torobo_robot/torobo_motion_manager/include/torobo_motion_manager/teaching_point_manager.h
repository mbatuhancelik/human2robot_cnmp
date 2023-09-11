/**
 * @file  teaching_point_manager.h
 * @brief Teaching point manager class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TEACHING_POINT_MANAGER_H
#define TEACHING_POINT_MANAGER_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <vector>
#include <thread>
#include "torobo_msgs/DeleteTeachingPoint.h"
#include "torobo_msgs/GetTeachingPoint.h"
#include "torobo_msgs/GetTeachingPointNames.h"
#include "torobo_msgs/RecordTeachingPoint.h"

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
namespace torobo
{

class TeachingPointManager
{
public:
    TeachingPointManager(ros::NodeHandle& nh,
        const std::string& controller_name,
        const std::vector<std::string>& joint_names,
        const double rate
    );
    virtual ~TeachingPointManager();

protected:
    bool DeleteTeachingPointService(torobo_msgs::DeleteTeachingPoint::Request &req, torobo_msgs::DeleteTeachingPoint::Response &res);
    bool GetTeachingPointService(torobo_msgs::GetTeachingPoint::Request &req, torobo_msgs::GetTeachingPoint::Response &res);
    bool GetTeachingPointNamesService(torobo_msgs::GetTeachingPointNames::Request &req, torobo_msgs::GetTeachingPointNames::Response &res);
    bool RecordTeachingPointService(torobo_msgs::RecordTeachingPoint::Request &req, torobo_msgs::RecordTeachingPoint::Response &res);

private:
    void queueThread();
    std::vector<std::string> GetTeachingPointNames();
    
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
