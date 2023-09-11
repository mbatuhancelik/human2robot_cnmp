#include "torobo_collision_detector/torobo_collision_detector_nodecore.h"
#include <nodelet/nodelet.h>

using namespace std;

namespace torobo
{

class ToroboCollisionDetectorNodelet : public nodelet::Nodelet
{
private:
    std::unique_ptr<ToroboCollisionDetectorNodeCore> node_;

public:
    ToroboCollisionDetectorNodelet() :
        nodelet::Nodelet(),
        node_(nullptr)
    {
    }

    ~ToroboCollisionDetectorNodelet()
    {
    }

    void onInit() override
    {
        ros::NodeHandle& nh = this->getNodeHandle();
        ros::NodeHandle& nh_private = this->getPrivateNodeHandle();
        node_.reset(new ToroboCollisionDetectorNodeCore(nh, nh_private));
    }
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(torobo::ToroboCollisionDetectorNodelet, nodelet::Nodelet);
