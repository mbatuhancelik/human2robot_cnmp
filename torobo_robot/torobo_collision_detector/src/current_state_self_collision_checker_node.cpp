#include "torobo_collision_detector/current_state_self_collision_checker_nodecore.h"

int main(int argc, char **argv)
{
    ros::init (argc, argv, "current_state_self_collision_checker_node");
    torobo::CurrentStateSelfCollisionCheckerNodeCore node(
        ros::NodeHandle(), ros::NodeHandle("~"));
    node.run();

    return 0;
}