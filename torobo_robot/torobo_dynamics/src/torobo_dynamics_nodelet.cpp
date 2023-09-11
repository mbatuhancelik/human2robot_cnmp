#include "torobo_dynamics/torobo_dynamics_nodecore.h"
#include <nodelet/nodelet.h>

using namespace std;

namespace torobo
{

class ToroboDynamicsNodelet : public nodelet::Nodelet
{
private:
    std::unique_ptr<ToroboDynamicsNodeCore> node_;
    ros::Timer timer_;

public:
    ToroboDynamicsNodelet() :
        nodelet::Nodelet(),
        node_(nullptr)
    {
    }

    ~ToroboDynamicsNodelet()
    {
    }

    void onInit() override
    {
        ros::NodeHandle& nh = this->getNodeHandle();
        ros::NodeHandle& nh_private = this->getPrivateNodeHandle();
        node_.reset(new ToroboDynamicsNodeCore(nh, nh_private));
        timer_ = nh.createTimer(ros::Duration(1.0 / node_->getRate()),
          [&](const ros::TimerEvent& te) {this->node_->runOnce();});
    }
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(torobo::ToroboDynamicsNodelet, nodelet::Nodelet);
