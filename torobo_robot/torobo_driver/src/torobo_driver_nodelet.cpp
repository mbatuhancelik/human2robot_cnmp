#include "torobo_driver/torobo_driver_nodecore.h"
#include <nodelet/nodelet.h>

using namespace std;

namespace torobo
{

class ToroboDriverNodelet : public nodelet::Nodelet
{
private:
    std::unique_ptr<ToroboDriverNodeCore> node_;
    std::unique_ptr<std::thread> thread_;

public:
    ToroboDriverNodelet() :
        nodelet::Nodelet(),
        node_(nullptr),
        thread_(nullptr)
    {
    }

    ~ToroboDriverNodelet()
    {
        thread_->join();
    }

    void onInit() override
    {
        ros::NodeHandle& nh = this->getNodeHandle();
        ros::NodeHandle& nh_private = this->getPrivateNodeHandle();
        node_.reset(new ToroboDriverNodeCore(nh, nh_private));
        thread_.reset(new std::thread(std::bind(&ToroboDriverNodeCore::run, node_.get())));
    }
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(torobo::ToroboDriverNodelet, nodelet::Nodelet);
