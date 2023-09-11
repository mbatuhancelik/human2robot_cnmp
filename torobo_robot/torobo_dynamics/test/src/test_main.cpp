#include <ros/ros.h>
#include <gtest/gtest.h>

int	main(int argc, char **argv)
{
    ros::init(argc,argv, "test_torobo_dynamics_node");
	testing::InitGoogleTest(&argc, argv);
	return  RUN_ALL_TESTS();
}