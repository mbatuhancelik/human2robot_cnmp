#include "torobo_motion_manager/torobo_motion_manager_nodecore.h"
#include <nodelet/nodelet.h>

using namespace std;

namespace torobo
{

class ToroboMotionManagerNodelet : public nodelet::Nodelet
{
private:
    std::unique_ptr<ToroboMotionManagerNodeCore> node_;

public:
    ToroboMotionManagerNodelet() :
        nodelet::Nodelet(),
        node_(nullptr)
    {
    }

    ~ToroboMotionManagerNodelet()
    {
    }

    void onInit() override
    {
        ros::NodeHandle& nh = this->getNodeHandle();
        ros::NodeHandle& nh_private = this->getPrivateNodeHandle();
        node_.reset(new ToroboMotionManagerNodeCore(nh, nh_private));
    }
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(torobo::ToroboMotionManagerNodelet, nodelet::Nodelet);
