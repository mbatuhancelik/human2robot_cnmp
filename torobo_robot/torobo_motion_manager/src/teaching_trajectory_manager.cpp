/**
 * @file  teaching_trajectory_manager.cpp
 * @brief Teaching trajectory manager class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "torobo_motion_manager/teaching_trajectory_manager.h"
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

TeachingTrajectoryManager::TeachingTrajectoryManager(ros::NodeHandle& nh,
    const std::string& controller_name,
    const std::vector<std::string>& joint_names,
    const double rate
)
    : nh_(nh), controller_name_(controller_name), joint_names_(joint_names), duration_(1.0 / rate)
{
    ros::AdvertiseServiceOptions del_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::DeleteTeachingTrajectory>(
        controller_name_ + "/delete_teaching_trajectory",
        boost::bind(&TeachingTrajectoryManager::DeleteTeachingTrajectoryService, this, _1, _2),
        ros::VoidPtr(), 
        &cb_queue_
    );
    services_.emplace_back(nh_.advertiseService(del_aso));

    ros::AdvertiseServiceOptions get_tp_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::GetTeachingTrajectory>(
        controller_name_ + "/get_teaching_trajectory",
        boost::bind(&TeachingTrajectoryManager::GetTeachingTrajectoryService, this, _1, _2),
        ros::VoidPtr(), 
        &cb_queue_
    );
    services_.emplace_back(nh_.advertiseService(get_tp_aso));

    ros::AdvertiseServiceOptions get_tp_names_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::GetTeachingTrajectoryNames>(
        controller_name_ + "/get_teaching_trajectory_names",
        boost::bind(&TeachingTrajectoryManager::GetTeachingTrajectoryNamesService, this, _1, _2),
        ros::VoidPtr(), 
        &cb_queue_
    );
    services_.emplace_back(nh_.advertiseService(get_tp_names_aso));

    ros::AdvertiseServiceOptions record_tp_aso = ros::AdvertiseServiceOptions::create<torobo_msgs::RecordTeachingTrajectory>(
        controller_name_ + "/record_teaching_trajectory",
        boost::bind(&TeachingTrajectoryManager::RecordTeachingTrajectoryService, this, _1, _2),
        ros::VoidPtr(), 
        &cb_queue_
    );
    services_.emplace_back(nh_.advertiseService(record_tp_aso));

    thread_.reset(new thread(std::bind(&TeachingTrajectoryManager::queueThread, this)));
}

TeachingTrajectoryManager::~TeachingTrajectoryManager()
{
    thread_working_ = false;
    thread_->join();
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
bool TeachingTrajectoryManager::DeleteTeachingTrajectoryService(torobo_msgs::DeleteTeachingTrajectory::Request &req, torobo_msgs::DeleteTeachingTrajectory::Response &res)
{
    const string paramName = controller_name_ + "/teaching_trajectories/" + req.teachingTrajectoryName;
    const string nameParamName = controller_name_ + "/teaching_trajectories/names";
    if(!nh_.hasParam(paramName))
    {
        res.success = false;
        return true;
    }
    nh_.deleteParam(paramName);

    // delete name from teaching_trajectory/names
    vector<string> names = GetTeachingTrajectoryNames();
    set<string> setnames(names.begin(), names.end());
    setnames.erase(req.teachingTrajectoryName);
    vector<string> newnames(setnames.begin(), setnames.end());
    nh_.setParam(nameParamName, newnames);

    res.success = true;
    return true;
}

static double parseXmlRpcValueToDouble(XmlRpc::XmlRpcValue& value)
{
    if(value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
        return 0.0;
    }
    return static_cast<double>(value);
}

static vector<double> parseXmlRpcValueToVector(XmlRpc::XmlRpcValue& value)
{
    vector<double> v;
    if(value.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        return v;
    }
    for(int i = 0; i < value.size(); i++)
    {
        if(value[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
        {
            v.clear();
            break;
        }
        const double d = static_cast<double>(value[i]);
        v.push_back(d);
    }
    return move(v);
}

bool TeachingTrajectoryManager::GetTeachingTrajectoryService(torobo_msgs::GetTeachingTrajectory::Request &req, torobo_msgs::GetTeachingTrajectory::Response &res)
{
    const string paramName = controller_name_ + "/teaching_trajectories/" + req.teachingTrajectoryName;
    if(!nh_.hasParam(paramName))
    {
        res.success = false;
        return true;
    }

    XmlRpc::XmlRpcValue trajectory_points;
    nh_.getParam(paramName, trajectory_points);

    for(int i = 0; i < trajectory_points.size(); i++)
    {
        trajectory_msgs::JointTrajectoryPoint point;
        for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator itr = trajectory_points[i].begin(); itr != trajectory_points[i].end(); ++itr)
        {
            const string& name = itr->first;
            XmlRpc::XmlRpcValue value = itr->second;
            if(name == "positions")
            {
                point.positions = parseXmlRpcValueToVector(value);
            }
            else if(name == "velocities")
            {
                point.velocities = parseXmlRpcValueToVector(value);
            }
            else if(name == "accelerations")
            {
                point.accelerations = parseXmlRpcValueToVector(value);
            }
            else if(name == "effort")
            {
                point.effort = parseXmlRpcValueToVector(value);
            }
            else if(name == "time_from_start")
            {
                point.time_from_start = ros::Duration(parseXmlRpcValueToDouble(value));
            }
        }
        res.trajectory.points.push_back(point);
    }
    res.trajectory.header.stamp = ros::Time::now();
    res.trajectory.joint_names = joint_names_;

    res.success = true;
    return true;
}

bool TeachingTrajectoryManager::GetTeachingTrajectoryNamesService(torobo_msgs::GetTeachingTrajectoryNames::Request &req, torobo_msgs::GetTeachingTrajectoryNames::Response &res)
{
    vector<string> names = GetTeachingTrajectoryNames();
    if(names.size() == 0)
    {
        res.success = false;
        return true;
    }
    res.success = true;
    res.teachingTrajectoryNames = names;
    return true;
}

bool TeachingTrajectoryManager::RecordTeachingTrajectoryService(torobo_msgs::RecordTeachingTrajectory::Request &req, torobo_msgs::RecordTeachingTrajectory::Response &res)
{
    const string trajParamName = controller_name_ + "/teaching_trajectories/" + req.teachingTrajectoryName;
    const string nameParamName = controller_name_ + "/teaching_trajectories/names";
    // ROS_INFO_STREAM("RecordTraj[" << controller_name_ << "] : thread id = " << std::this_thread::get_id());

    // duplicate cut
    vector<string> names = GetTeachingTrajectoryNames();
    set<string> setnames(names.begin(), names.end());
    setnames.insert(req.teachingTrajectoryName);
    vector<string> newnames(setnames.begin(), setnames.end());

    // create trajectory XmlRpcValue
    XmlRpc::XmlRpcValue trajectory_points;
    for(int i = 0; i < req.trajectory.points.size(); i++)
    {
        XmlRpc::XmlRpcValue point;
        XmlRpc::XmlRpcValue positions, velocities, accelerations, effort;
        for(int j = 0; j < req.trajectory.points[i].positions.size(); j++)
        {
            positions[j] = req.trajectory.points[i].positions[j];
            velocities[j] = req.trajectory.points[i].velocities[j];
            accelerations[j] = req.trajectory.points[i].accelerations[j];
            effort[j] = req.trajectory.points[i].effort[j];
        }
        point["positions"] = positions;
        point["velocities"] = velocities;
        point["accelerations"] = accelerations;
        point["effort"] = effort;
        point["time_from_start"] = req.trajectory.points[i].time_from_start.toSec();
        trajectory_points[i] = point;
    }
    nh_.setParam(trajParamName, trajectory_points);
    nh_.setParam(nameParamName, newnames);

    res.success = true;
    return true;
}

/*----------------------------------------------------------------------
 Private Method Definitions
 ----------------------------------------------------------------------*/
void TeachingTrajectoryManager::queueThread()
{
    thread_working_ = true;
    while(thread_working_)
    {
        cb_queue_.callAvailable(ros::WallDuration(duration_));
    }
}

std::vector<std::string> TeachingTrajectoryManager::GetTeachingTrajectoryNames()
{
   vector<string> names;
   names.clear();

   const string paramName = controller_name_ + "/teaching_trajectories/names";
   if(nh_.hasParam(paramName))
   {
       nh_.getParam(paramName, names);
   }
   return names;
}

}