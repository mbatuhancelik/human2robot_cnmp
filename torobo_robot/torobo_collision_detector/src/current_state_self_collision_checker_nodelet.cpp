#include "torobo_collision_detector/current_state_self_collision_checker_nodecore.h"
#include <nodelet/nodelet.h>

using namespace std;

namespace torobo
{

class CurrentStateSelfCollisionCheckerNodelet : public nodelet::Nodelet
{
private:
    std::unique_ptr<CurrentStateSelfCollisionCheckerNodeCore> node_;
    ros::Timer timer_;

public:
    CurrentStateSelfCollisionCheckerNodelet() :
        nodelet::Nodelet(),
        node_(nullptr)
    {
    }

    ~CurrentStateSelfCollisionCheckerNodelet()
    {
    }

    void onInit() override
    {
        ros::NodeHandle& nh = this->getNodeHandle();
        ros::NodeHandle& nh_private = this->getPrivateNodeHandle();
        node_.reset(new CurrentStateSelfCollisionCheckerNodeCore(nh, nh_private));
        if(node_->isInit())
        {
            timer_ = nh.createTimer(ros::Duration(1.0 / node_->getRate()),
              [&](const ros::TimerEvent& te) {this->node_->runOnce();});
        }
    }
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(torobo::CurrentStateSelfCollisionCheckerNodelet, nodelet::Nodelet);
