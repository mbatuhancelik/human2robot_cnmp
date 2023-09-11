#include "torobo_motion_manager/torobo_motion_manager_nodecore.h"
#include "torobo_common/rosparam_loader.h"

using namespace std;

namespace torobo
{

ToroboMotionManagerNodeCore::ToroboMotionManagerNodeCore(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh)
{
    // get params
    private_nh_.param<bool>("debug", debug_, false);
    private_nh_.param<double>("rate", rate_, 50.0);
    private_nh_.param<double>("timeout_sec", timeout_sec_, 5.0);

    // initialize
    map<string, vector<string>> controller_joints_map;
    if(!torobo_common::getControllerJointsMap(controller_joints_map, nh))
    {
        ROS_ERROR("not found any valid controller. failed to init [%s].", ros::this_node::getName().c_str());
        return;
    }

    if(debug_)
    {
        for(auto it = controller_joints_map.begin(); it != controller_joints_map.end(); ++it)
        {
            cout << "found controller name: " << it->first << endl;
            for(auto jit = it->second.begin(); jit != it->second.end(); ++jit)
            {
                cout << "    joint: " << *jit << endl;
            }
        }
    }

    try
    {
        for(auto it = controller_joints_map.begin(); it != controller_joints_map.end(); ++it)
        {
            const string& controller_name = it->first;
            if(controller_name.find("gripper") != string::npos)
            {
                continue;
            }
            tpm_.emplace_back(new TeachingPointManager(nh_, controller_name, it->second, rate_));
            ttm_.emplace_back(new TeachingTrajectoryManager(nh_, controller_name, it->second, rate_));
            move_home_as_.emplace_back(new MoveHomePositionActionServer(nh, controller_name, it->second, rate_, timeout_sec_, debug_));
            move_tp_as_.emplace_back(new MoveTeachingPointActionServer(nh, controller_name, it->second, rate_, timeout_sec_, debug_));
            move_tt_as_.emplace_back(new MoveTeachingTrajectoryActionServer(nh, controller_name, it->second, rate_, timeout_sec_, debug_));
        }
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM(e.what());
        return;
    }

    ROS_INFO("ToroboMotionManagerNodeCore Ready.");
}

ToroboMotionManagerNodeCore::~ToroboMotionManagerNodeCore()
{
}

void ToroboMotionManagerNodeCore::run()
{
    while(ros::ok())
    {
        ros::spin();
    }
}

}