#include "torobo_collision_detector/torobo_collision_detector_nodecore.h"

int main(int argc, char **argv)
{
    ros::init (argc, argv, "torobo_collision_detector_node");
    torobo::ToroboCollisionDetectorNodeCore node(
        ros::NodeHandle(), ros::NodeHandle("~"));
    node.run();

    return 0;
}