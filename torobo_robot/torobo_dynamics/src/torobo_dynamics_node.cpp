#include "torobo_dynamics/torobo_dynamics_nodecore.h"

int main(int argc, char **argv)
{
    ros::init (argc, argv, "torobo_dynamics_node");
    torobo::ToroboDynamicsNodeCore node(
        ros::NodeHandle(), ros::NodeHandle("~"));
    node.run();

    return 0;
}