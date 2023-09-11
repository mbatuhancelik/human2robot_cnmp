#include "torobo_dynamics/torobo_dynamics_nodecore.h"
#include "torobo_common/rosparam_loader.h"

using namespace std;

namespace torobo {

ToroboDynamicsNodeCore::ToroboDynamicsNodeCore(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh), is_init_(false)
{
    // get params
    private_nh_.param<bool>("debug", debug_, false);
    private_nh_.param<bool>("sim", sim_, true);
    private_nh_.param<double>("rate", rate_, 100.0);
    nh_.param<vector<string>>("controller_list", controller_list_, vector<string>());

    // initialize
    map<string, vector<string>> controller_joints_map;
    if(!torobo_common::getControllerJointsMap(controller_joints_map, nh))
    {
        ROS_ERROR("not found any valid controller. failed to init ToroboDynamicsNodeCore");
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

    torobo_state_.reset(new torobo::ToroboState(nh_, controller_joints_map, sim_));
    torobo_dyna_.reset(new torobo::ToroboDynamics(nh_, controller_joints_map, sim_));
    is_init_ = true;

    ROS_INFO("ToroboDynamicsNodeCore Ready.");
}

ToroboDynamicsNodeCore::~ToroboDynamicsNodeCore()
{
}

void ToroboDynamicsNodeCore::run()
{
    ros::Rate loop(rate_);

    while(ros::ok())
    {
        ros::spinOnce();
        runOnce();
        loop.sleep();
    };
}

void ToroboDynamicsNodeCore::runOnce()
{
    if(is_init_)
    {
        torobo_dyna_->Update(torobo_state_);
        torobo_dyna_->Publish(); 
    }
}

}