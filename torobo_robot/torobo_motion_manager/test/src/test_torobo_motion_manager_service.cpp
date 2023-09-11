#include <ros/ros.h>
#include <gtest/gtest.h>
#include <memory>
#include "torobo_motion_manager/teaching_point_manager.h"
#include "torobo_motion_manager/teaching_trajectory_manager.h"

using namespace std;
using namespace torobo;

int	main(int argc, char **argv)
{
    ros::init(argc, argv, "test_torobo_motion_manager_service_node");
	testing::InitGoogleTest(&argc, argv);
	return  RUN_ALL_TESTS();
}

struct ControllerServiceHelper
{
    void assertWaitForService(const string& controller_name, ros::Duration timeout=ros::Duration(1.0))
    {
        ASSERT_TRUE(ros::service::waitForService(controller_name + "/delete_teaching_point", timeout));
        ASSERT_TRUE(ros::service::waitForService(controller_name + "/get_teaching_point", timeout));
        ASSERT_TRUE(ros::service::waitForService(controller_name + "/get_teaching_point_names", timeout));
        ASSERT_TRUE(ros::service::waitForService(controller_name + "/record_teaching_point", timeout));
        ASSERT_TRUE(ros::service::waitForService(controller_name + "/delete_teaching_trajectory", timeout));
        ASSERT_TRUE(ros::service::waitForService(controller_name + "/get_teaching_trajectory", timeout));
        ASSERT_TRUE(ros::service::waitForService(controller_name + "/get_teaching_trajectory_names", timeout));
        ASSERT_TRUE(ros::service::waitForService(controller_name + "/record_teaching_trajectory", timeout));
    }
};

class ToroboMotionManagerServiceTestFixture : public ::testing::Test
{
public:
    // member
    ros::NodeHandle nh_;
    std::vector<std::unique_ptr<TeachingPointManager>> tpm_;
    std::vector<std::unique_ptr<TeachingTrajectoryManager>> ttm_;

    const double rate_;

    ToroboMotionManagerServiceTestFixture()
        : rate_(100)
    {
    }

    virtual void SetUp()
    {
    }
    virtual void TearDown()
    {
        tpm_.clear();
        ttm_.clear();
    }

    vector<string> getJointNames(int joint_size, const string& joint_prefix="")
    {
        vector<string> joint_names;
        for(int i = 0; i < joint_size; i++)
        {
            string joint_name = joint_prefix + "joint";
            if(joint_size > 1)
            {
                joint_name += "_" + to_string(i+1);
            }
            joint_names.push_back(joint_name);
        }
        return joint_names;
    }

    void createSingleArm(int joint_size)
    {
        std::string controller_name = "arm_controller";
        std::vector<std::string> joint_names = getJointNames(joint_size, "arm/");
        tpm_.emplace_back(new TeachingPointManager(nh_, controller_name, joint_names, rate_));
        ttm_.emplace_back(new TeachingTrajectoryManager(nh_, controller_name, joint_names, rate_));
    }

    void createHumanoid(int left_arm_joint_size, int right_arm_joint_size, int torso_joint_size, int head_joint_size)
    {
        map<string, int> prefix_joint_size_map = {
            {"left_arm", left_arm_joint_size},
            {"right_arm",  right_arm_joint_size},
            {"torso",  torso_joint_size},
            {"head",  head_joint_size}
        };
        for(auto itr : prefix_joint_size_map)
        {
            std::string controller_name = itr.first + "_controller";
            std::vector<std::string> joint_names = getJointNames(itr.second, itr.first + "/");
            tpm_.emplace_back(new TeachingPointManager(nh_,      controller_name, joint_names, rate_));
            ttm_.emplace_back(new TeachingTrajectoryManager(nh_, controller_name, joint_names, rate_));
        }
    }
};

TEST_F(ToroboMotionManagerServiceTestFixture, 7AxisSingleArmSrvsExist)
{
    createSingleArm(7);

    // create service helper
    ControllerServiceHelper srv_h;

    // check service exists
    srv_h.assertWaitForService("arm_controller");
}

TEST_F(ToroboMotionManagerServiceTestFixture, 16AxisHumanoidSrvsExist)
{
    createHumanoid(6, 6, 2, 2);

    // create service helper
    ControllerServiceHelper srv_h;

    // check service exists
    srv_h.assertWaitForService("left_arm_controller");
    srv_h.assertWaitForService("right_arm_controller");
    srv_h.assertWaitForService("torso_controller");
    srv_h.assertWaitForService("head_controller");
}
