#include "torobo_driver/torobo_driver_nodecore.h"

int main(int argc, char **argv)
{
    ros::init (argc, argv, "torobo_driver_node");
    torobo::ToroboDriverNodeCore node(
        ros::NodeHandle(), ros::NodeHandle("~"));
    node.run();
    ros::waitForShutdown();

    return 0;
}