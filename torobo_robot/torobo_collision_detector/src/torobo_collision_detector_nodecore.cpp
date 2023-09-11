#include "torobo_collision_detector/torobo_collision_detector_nodecore.h"

using namespace std;

namespace torobo
{

ToroboCollisionDetectorNodeCore::ToroboCollisionDetectorNodeCore(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh)
{

    // get params
    private_nh_.param<bool>("debug", debug_, false);
    private_nh_.param<string>("robot_description_name", robot_description_name_, "robot_description");
    private_nh_.param<string>("service_name_for_check_collision", service_name_for_check_collision_, "check_collision");
    private_nh_.param<string>("service_name_for_get_collision_info", service_name_for_get_collision_info_, "get_collision_info");
    private_nh_.param<double>("link_scale", link_scale_, 1.0);  // note: if link_scale is changed from 1.0, it takes calculation time consumption.
    private_nh_.param<string>("link_name_regex", link_name_regex_, ".*");

    detector_.reset(new ToroboCollisionDetector(nh_, robot_description_name_,
        service_name_for_check_collision_, service_name_for_get_collision_info_, link_scale_, link_name_regex_, debug_));
    ROS_INFO("ToroboCollisionDetectorNodeCore Ready.");
}

ToroboCollisionDetectorNodeCore::~ToroboCollisionDetectorNodeCore()
{
}

void ToroboCollisionDetectorNodeCore::run()
{
    while(ros::ok())
    {
        ros::spin();
    }
}

}