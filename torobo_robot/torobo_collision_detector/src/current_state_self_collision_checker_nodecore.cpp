#include "torobo_collision_detector/current_state_self_collision_checker_nodecore.h"
#include "torobo_common/rosparam_loader.h"

using namespace std;

namespace torobo
{

CurrentStateSelfCollisionCheckerNodeCore::CurrentStateSelfCollisionCheckerNodeCore(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh)
{
    // get params
    private_nh_.param<bool>("sim", sim_, false);
    private_nh_.param<double>("rate", rate_, 100);
    private_nh_.param<std::string>("service_name_for_check_collision", service_name_for_check_collision_, "check_collision");

    std::map<std::string, std::vector<std::string>> controller_joints_map;
    if(!torobo_common::getControllerJointsMap(controller_joints_map, nh_))
    {
        ROS_FATAL("failed to get controller joints map. shutdown [%s].", ros::this_node::getName().c_str());
        ros::shutdown();
    }

    checker_.reset(new CurrentStateSelfCollisionChecker(nh_, controller_joints_map, sim_, service_name_for_check_collision_));
    if(!checker_->isInit())
    {
        ROS_FATAL("Failed to initialize [CurrentStateSelfCollisionCheckerNodeCore]. shutdown [%s]", ros::this_node::getName().c_str());
        ros::shutdown();
    }
    is_init_ = checker_->isInit();
    ROS_INFO("current state self collision checker standby!");
}

CurrentStateSelfCollisionCheckerNodeCore::~CurrentStateSelfCollisionCheckerNodeCore()
{
}

void CurrentStateSelfCollisionCheckerNodeCore::run()
{
    ros::Rate loop(rate_);
    while(ros::ok())
    {
        runOnce();
        loop.sleep();
    }
}

void CurrentStateSelfCollisionCheckerNodeCore::runOnce()
{
    checker_->checkSelfCollision();
}

}
