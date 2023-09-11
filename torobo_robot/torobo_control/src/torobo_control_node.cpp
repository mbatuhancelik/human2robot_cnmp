/**
 * @file  torobo_control_node.cpp
 * @brief 
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include "torobo_control/torobo_control_nodecore.h"

int main(int argc, char **argv)
{
    ros::init (argc, argv, "torobo_control_node");
    torobo::ToroboControlNodeCore node(
        ros::NodeHandle(), ros::NodeHandle("~"));
    node.run();
    ros::spin();

    return 0;
}
