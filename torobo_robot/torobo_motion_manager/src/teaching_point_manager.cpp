/**
 * @file  teaching_point_manager.cpp
 * @brief Teaching point manager class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "torobo_motion_manager/teaching_point_manager.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <set>
#include <math.h>

using namespace std;

/*----------------------------------------------------------------------
 Privat Global Variables
 ----------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
namespace torobo
{

TeachingPointManager::TeachingPointManager(ros::NodeHandle& nh,
    const std::string& controller_name,
    const std::vector<std::string>& joint_names,
    const double rate
)
    : nh_(nh), controller_name_(controller_name), joint_names_(joint_names), duration_(1.0 / rate)
{
    ros::AdvertiseServiceOptions del_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::DeleteTeachingPoint>(
        controller_name_ + "/delete_teaching_point",
        boost::bind(&TeachingPointManager::DeleteTeachingPointService, this, _1, _2),
        ros::VoidPtr(), 
        &cb_queue_
    );
    services_.emplace_back(nh_.advertiseService(del_aso));

    ros::AdvertiseServiceOptions get_tp_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::GetTeachingPoint>(
        controller_name_ + "/get_teaching_point",
        boost::bind(&TeachingPointManager::GetTeachingPointService, this, _1, _2),
        ros::VoidPtr(), 
        &cb_queue_
    );
    services_.emplace_back(nh_.advertiseService(get_tp_aso));

    ros::AdvertiseServiceOptions get_tp_names_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::GetTeachingPointNames>(
        controller_name_ + "/get_teaching_point_names",
        boost::bind(&TeachingPointManager::GetTeachingPointNamesService, this, _1, _2),
        ros::VoidPtr(), 
        &cb_queue_
    );
    services_.emplace_back(nh_.advertiseService(get_tp_names_aso));

    ros::AdvertiseServiceOptions record_tp_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::RecordTeachingPoint>(
        controller_name_ + "/record_teaching_point",
        boost::bind(&TeachingPointManager::RecordTeachingPointService, this, _1, _2),
        ros::VoidPtr(), 
        &cb_queue_
    );
    services_.emplace_back(nh_.advertiseService(record_tp_aso));

    thread_.reset(new thread(std::bind(&TeachingPointManager::queueThread, this)));
}

TeachingPointManager::~TeachingPointManager()
{
    thread_working_ = false;
    thread_->join();
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
bool TeachingPointManager::DeleteTeachingPointService(torobo_msgs::DeleteTeachingPoint::Request &req, torobo_msgs::DeleteTeachingPoint::Response &res)
{
    const string paramName = controller_name_ + "/teaching_points/" + req.teachingPointName;
    const string nameParamName = controller_name_ + "/teaching_points/names";
    if(!nh_.hasParam(paramName))
    {
        res.success = false;
        return true;
    }
    nh_.deleteParam(paramName);

    // delete name from teaching_point/names
    vector<string> names = GetTeachingPointNames();
    set<string> setnames(names.begin(), names.end());
    setnames.erase(req.teachingPointName);
    vector<string> newnames(setnames.begin(), setnames.end());
    nh_.setParam(nameParamName, newnames);

    res.success = true;
    return true;
}

bool TeachingPointManager::GetTeachingPointService(torobo_msgs::GetTeachingPoint::Request &req, torobo_msgs::GetTeachingPoint::Response &res)
{
    const string paramName = controller_name_ + "/teaching_points/" + req.teachingPointName;
    if(!nh_.hasParam(paramName))
    {
        res.success = false;
        return true;
    }
    vector<double> positions;
    nh_.getParam(paramName, positions);

    for(int i = 0; i < positions.size(); i++)
    {
        res.point.positions.push_back(positions[i]);
        res.point.velocities.push_back(0.0f);
        res.point.accelerations.push_back(0.0f);
        res.point.effort.push_back(0.0f);
    }
    res.success = true;
    return true;
}

bool TeachingPointManager::GetTeachingPointNamesService(torobo_msgs::GetTeachingPointNames::Request &req, torobo_msgs::GetTeachingPointNames::Response &res)
{
    vector<string> names = GetTeachingPointNames();
    if(names.size() == 0)
    {
        res.success = false;
        return true;
    }
    res.success = true;
    res.teachingPointNames = names;
    return true;
}

bool TeachingPointManager::RecordTeachingPointService(torobo_msgs::RecordTeachingPoint::Request &req, torobo_msgs::RecordTeachingPoint::Response &res)
{
    const string posParamName = controller_name_ + "/teaching_points/" + req.teachingPointName;
    const string nameParamName = controller_name_ + "/teaching_points/names";
    // ROS_INFO_STREAM("RecordTP[" << controller_name_ << "] : thread id = " << std::this_thread::get_id());

    vector<double> positions(req.point.positions);
    vector<string> names = GetTeachingPointNames();
    // duplicate cut
    set<string> setnames(names.begin(), names.end());
    setnames.insert(req.teachingPointName);
    vector<string> newnames(setnames.begin(), setnames.end());

    nh_.setParam(posParamName, positions);
    nh_.setParam(nameParamName, newnames);
    res.success = true;
    return true;
}

/*----------------------------------------------------------------------
 Private Method Definitions
 ----------------------------------------------------------------------*/
void TeachingPointManager::queueThread()
{
    thread_working_ = true;
    while(thread_working_)
    {
        cb_queue_.callAvailable(ros::WallDuration(duration_));
    }
}

std::vector<std::string> TeachingPointManager::GetTeachingPointNames()
{
   vector<string> names;
   names.clear();

   const string paramName = controller_name_ + "/teaching_points/names";
   if(nh_.hasParam(paramName))
   {
       nh_.getParam(paramName, names);
   }
   return names;
}

}