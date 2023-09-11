/**
 * @file  torobo_control_nodelet.cpp
 * @brief ToroboControlNodelet class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include "torobo_control/torobo_control_nodecore.h"
#include "nodelet/nodelet.h"


using namespace std;

namespace torobo
{

class ToroboControlNodelet: public nodelet::Nodelet
{
private:
    std::unique_ptr<ToroboControlNodeCore> node_;

public:
    ToroboControlNodelet() :
        nodelet::Nodelet(),
        node_(nullptr)
    {
    }

    ~ToroboControlNodelet()
    {
    }

    void onInit() override
    {
        ros::NodeHandle& nh = this->getNodeHandle();
        ros::NodeHandle& nh_private = this->getPrivateNodeHandle();
        node_.reset(new ToroboControlNodeCore(nh, nh_private));
        node_->run();
    }
};

} // namespace torobo

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(torobo::ToroboControlNodelet, nodelet::Nodelet);
