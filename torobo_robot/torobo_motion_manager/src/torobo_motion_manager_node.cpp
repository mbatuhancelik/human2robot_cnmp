#include "torobo_motion_manager/torobo_motion_manager_nodecore.h"

int main(int argc, char **argv)
{
    ros::init (argc, argv, "torobo_motion_manager_node");
    torobo::ToroboMotionManagerNodeCore node(
        ros::NodeHandle(), ros::NodeHandle("~"));
    node.run();

    return 0;
}