#include <ros/ros.h>
#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/GripperCommand.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "torobo_driver/torobo_driver.h"
#include "torobo_msgs/ToroboJointState.h"

using namespace std;
using namespace torobo;


struct ToroboJointStateSubHelper
{
    ToroboJointStateSubHelper()
        : count_(0)
    {
    }

    void cb(const torobo_msgs::ToroboJointState::ConstPtr& msg)
    {
        count_++;
        state_ = *msg;
    }

    torobo_msgs::ToroboJointState state_;
    int count_;
};

struct JointStateSubHelper
{
    JointStateSubHelper()
        : count_(0)
    {
    }

    void cb(const sensor_msgs::JointState::ConstPtr& msg)
    {
        count_++;
        state_ = *msg;
    }

    sensor_msgs::JointState state_;
    int count_;
};

struct JointTrajectoryControllerStateSubHelper
{
    JointTrajectoryControllerStateSubHelper()
        : count_(0)
    {
    }

    void cb(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
    {
        count_++;
        state_ = *msg;
    }

    control_msgs::JointTrajectoryControllerState state_;
    int count_;
};

struct ControllerSubscriberHelper
{
    ControllerSubscriberHelper(ros::NodeHandle& nh, const vector<string>& controller_names)
        : controller_names_(controller_names)
    {
        for(auto controller_name : controller_names)
        {
            tjs_h_.insert(make_pair(controller_name, ToroboJointStateSubHelper()));
            js_h_.insert(make_pair(controller_name, JointStateSubHelper()));
            jtcs_h_.insert(make_pair(controller_name, JointTrajectoryControllerStateSubHelper()));
            subs_.emplace_back(nh.subscribe(controller_name + "/torobo_joint_state", 1, &ToroboJointStateSubHelper::cb, &tjs_h_.at(controller_name)));
            subs_.emplace_back(nh.subscribe(controller_name + "/joint_state", 1, &JointStateSubHelper::cb, &js_h_.at(controller_name)));
            subs_.emplace_back(nh.subscribe(controller_name + "/state", 1, &JointTrajectoryControllerStateSubHelper::cb, &jtcs_h_.at(controller_name)));
        }
    }

    void assertExpectedGetNumPublishers(const uint expected_publishers_num)
    {
        for(auto sub : subs_)
        {
            EXPECT_EQ(sub.getNumPublishers(), expected_publishers_num);
        }
    }

    void assertExpectedCallCount(const string& controller_name, const int expected_call_count)
    {
        EXPECT_TRUE(tjs_h_.count(controller_name) > 0);
        EXPECT_TRUE(js_h_.count(controller_name) > 0);
        EXPECT_TRUE(jtcs_h_.count(controller_name) > 0);
        EXPECT_EQ(tjs_h_.at(controller_name).count_, expected_call_count);
        EXPECT_EQ(js_h_.at(controller_name).count_, expected_call_count);
        EXPECT_EQ(jtcs_h_.at(controller_name).count_, expected_call_count);
    }

    void assertExpectedReceiveState(const string& controller_name, const int expected_joint_size)
    {
        EXPECT_TRUE(tjs_h_.count(controller_name) > 0);
        EXPECT_TRUE(js_h_.count(controller_name) > 0);
        EXPECT_TRUE(jtcs_h_.count(controller_name) > 0);
        EXPECT_EQ(tjs_h_.at(controller_name).state_.name.size(), expected_joint_size);
        EXPECT_EQ(js_h_.at(controller_name).state_.name.size(), expected_joint_size);
        EXPECT_EQ(jtcs_h_.at(controller_name).state_.joint_names.size(), expected_joint_size);
    }

    vector<string> controller_names_;
    vector<ros::Subscriber> subs_;
    map<string, ToroboJointStateSubHelper> tjs_h_;
    map<string, JointStateSubHelper> js_h_;
    map<string, JointTrajectoryControllerStateSubHelper> jtcs_h_;
};

struct ControllerPublisherHelper
{
    ControllerPublisherHelper(ros::NodeHandle& nh, const vector<string>& controller_names)
        : controller_names_(controller_names)
    {
        for(auto controller_name : controller_names)
        {
            pubs_.emplace_back(nh.advertise<torobo_msgs::ToroboCommand>(controller_name + "/torobo_command", 0));
            if(controller_name.find("gripper") != std::string::npos)
            {
                pubs_.emplace_back(nh.advertise<control_msgs::GripperCommand>(controller_name + "/command", 0));
            }
            else
            {
                pubs_.emplace_back(nh.advertise<trajectory_msgs::JointTrajectory>(controller_name + "/command", 0));
                pubs_.emplace_back(nh.advertise<torobo_msgs::ToroboDynamics>(controller_name + "/torobo_dynamics", 0));
            }
        }
    }

    void assertExpectedGetNumSubscribers(const uint expected_subscribers_num)
    {
        for(auto pub : pubs_)
        {
            EXPECT_EQ(pub.getNumSubscribers(), expected_subscribers_num);
        }
    }

    vector<string> controller_names_;
    vector<ros::Publisher> pubs_;
};

struct ControllerServiceHelper
{
    void assertWaitForService(const string& controller_name, ros::Duration timeout=ros::Duration(2.0))
    {
        if(controller_name.find("gripper") != std::string::npos)
        {
            ASSERT_FALSE(ros::service::waitForService(controller_name + "/brake_off", timeout));
            ASSERT_FALSE(ros::service::waitForService(controller_name + "/brake_on", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/servo_off", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/servo_on", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/get_servo_state", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/error_reset", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/set_control_mode", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/set_zero_effort", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/clear_trajectory", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/cancel_trajectory", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/set_robot_controller_parameter", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/set_general_output_register", timeout));
            ASSERT_FALSE(ros::service::waitForService(controller_name + "/set_payload_param", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/send_common_command", timeout));
        }
        else
        {
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/brake_off", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/brake_on", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/servo_off", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/servo_on", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/get_servo_state", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/error_reset", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/set_control_mode", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/set_zero_effort", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/clear_trajectory", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/cancel_trajectory", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/set_robot_controller_parameter", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/set_general_output_register", timeout));
            ASSERT_TRUE(ros::service::waitForService(controller_name + "/send_common_command", timeout));
            if(controller_name == "arm_controller")
            {
                ASSERT_TRUE(ros::service::waitForService(controller_name + "/set_payload_param", timeout));
            }
            else
            {
                ASSERT_FALSE(ros::service::waitForService(controller_name + "/set_payload_param", timeout));
            }
        }
    }
};

class ToroboDriverTestFixture : public ::testing::Test
{
public:
    // const value
    const chrono::nanoseconds period_ns_;

    // member
    ros::NodeHandle nh_;
    ToroboDriverCommonParam common_param_;

    ToroboDriverTestFixture()
        : period_ns_(static_cast<int64_t>(1e+9 / 1000))
    {
        common_param_.allowed_start_tolerance = 0.01;
    }

    virtual void SetUp()
    {
    }
    virtual void TearDown()
    {
    }

    pair<string, ControllerParam> getControllerParam(int joint_size, int id_offset = 0, const string& joint_type="revolute", const string& joint_prefix="", const string& controller_prefix="")
    {
        ControllerParam cparam;
        for(int i = 0; i < joint_size; i++)
        {
            string joint_name = joint_prefix + "joint";
            if(joint_size > 1)
            {
                joint_name += "_" + to_string(i+1);
            }
            cparam.joint_names.push_back(joint_name);
            cparam.joint_ids.push_back(i + id_offset);
            cparam.joint_types.push_back(joint_type);
        }
        return make_pair(controller_prefix + "controller" ,cparam);
    }

    unique_ptr<ToroboDriver> createSingleArm(int joint_size)
    {
        ToroboDriverParam param;
        param.controller.push_back(getControllerParam(joint_size, 0, "revolute", "arm/", "arm_"));
        param.allJointsNum = joint_size;
        return move(createDriver(param));
    }

    unique_ptr<ToroboDriver> createSingleArmGripper(int joint_size, const string& gripper_type="prismatic")
    {
        ToroboDriverParam param;
        param.controller.push_back(getControllerParam(joint_size, 0, "revolute", "arm/", "arm_"));
        param.controller.push_back(getControllerParam(1, joint_size, gripper_type, "finger_", "gripper_"));
        param.allJointsNum = joint_size + 1;
        return move(createDriver(param));
    }

    unique_ptr<ToroboDriver> createHumanoid(int left_arm_joint_size, int right_arm_joint_size, int torso_joint_size, int head_joint_size)
    {
        ToroboDriverParam param;
        param.controller.push_back(getControllerParam(left_arm_joint_size, 0, "revolute", "left_arm/", "left_arm_"));
        param.controller.push_back(getControllerParam(right_arm_joint_size, left_arm_joint_size, "revolute", "right_arm/", "right_arm_"));
        param.controller.push_back(getControllerParam(torso_joint_size, left_arm_joint_size + right_arm_joint_size, "revolute", "torso", "torso_"));
        param.controller.push_back(getControllerParam(head_joint_size, torso_joint_size + left_arm_joint_size + right_arm_joint_size, "revolute", "head", "head_"));
        param.allJointsNum = left_arm_joint_size + right_arm_joint_size + torso_joint_size + head_joint_size;
        return move(createDriver(param));
    }

    unique_ptr<ToroboDriver> createHumanoidGripper(int left_arm_joint_size, int right_arm_joint_size, int torso_joint_size, int head_joint_size, const string& gripper_type="prismatic")
    {
        ToroboDriverParam param;
        param.controller.push_back(getControllerParam(left_arm_joint_size, 0, "revolute", "left_arm/", "left_arm_"));
        param.controller.push_back(getControllerParam(right_arm_joint_size, left_arm_joint_size, "revolute", "right_arm/", "right_arm_"));
        param.controller.push_back(getControllerParam(torso_joint_size, left_arm_joint_size + right_arm_joint_size, "revolute", "torso", "torso_"));
        param.controller.push_back(getControllerParam(head_joint_size, torso_joint_size + left_arm_joint_size + right_arm_joint_size, "revolute", "head", "head_"));
        param.controller.push_back(getControllerParam(1, left_arm_joint_size + right_arm_joint_size + torso_joint_size + head_joint_size, gripper_type, "left_finger_", "left_gripper_"));
        param.controller.push_back(getControllerParam(1, left_arm_joint_size + right_arm_joint_size + torso_joint_size + head_joint_size + 1, gripper_type, "right_finger_", "right_gripper_"));
        param.allJointsNum = left_arm_joint_size + right_arm_joint_size + torso_joint_size + head_joint_size + 2;
        return move(createDriver(param));
    }

    unique_ptr<ToroboDriver> createDriver(const ToroboDriverParam& param, const int realtime_level=100, const double timeout_sec=3.0, bool is_mock=true)
    {
        unique_ptr<ToroboDriver> driver(new ToroboDriver(
            nh_, common_param_, param, period_ns_, realtime_level, timeout_sec, is_mock));
        driver->Initialize();
        return move(driver);
    }
};

TEST_F(ToroboDriverTestFixture, 7AxisSingleArmSubsExist)
{
    unique_ptr<ToroboDriver> driver(createSingleArm(7));
    ASSERT_TRUE(driver->IsInit());

    // create subscriber helper
    vector<string> controller_names {"arm_controller"};
    ControllerSubscriberHelper sub_h(nh_, controller_names);

    // check connected publishers num
    sub_h.assertExpectedGetNumPublishers(1U);

    // publish & subscribe
    driver->runOnce();
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // chech subscriber receive state
    sub_h.assertExpectedCallCount("arm_controller", 1U);
    sub_h.assertExpectedReceiveState("arm_controller", 7U);
}

TEST_F(ToroboDriverTestFixture, 7AxisSingleArmPubsExist)
{
    unique_ptr<ToroboDriver> driver(createSingleArm(7));
    ASSERT_TRUE(driver->IsInit());

    // create publisher helper
    vector<string> controller_names {"arm_controller"};
    ControllerPublisherHelper pub_h(nh_, controller_names);

    // check connected subscribers num
    pub_h.assertExpectedGetNumSubscribers(1U);
}

TEST_F(ToroboDriverTestFixture, 7AxisSingleArmSrvsExist)
{
    unique_ptr<ToroboDriver> driver(createSingleArm(7));
    ASSERT_TRUE(driver->IsInit());

    // create service helper
    ControllerServiceHelper srv_h;

    // check service exists
    srv_h.assertWaitForService("arm_controller");
}

TEST_F(ToroboDriverTestFixture, 7AxisSingleArmPrismaticGripperSubsExist)
{
    unique_ptr<ToroboDriver> driver(createSingleArmGripper(7, "prismatic"));
    ASSERT_TRUE(driver->IsInit());

    // create subscriber helper
    vector<string> controller_names {"arm_controller", "gripper_controller"};
    ControllerSubscriberHelper sub_h(nh_, controller_names);

    // check connected publishers num
    sub_h.assertExpectedGetNumPublishers(1U);

    // publish & subscribe
    driver->runOnce();
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // chech subscriber receive state
    sub_h.assertExpectedCallCount("arm_controller", 1U);
    sub_h.assertExpectedCallCount("gripper_controller", 1U);
    sub_h.assertExpectedReceiveState("arm_controller", 7U);
    sub_h.assertExpectedReceiveState("gripper_controller", 1U);
}

TEST_F(ToroboDriverTestFixture, 7AxisSingleArmPrismaticGripperPubsExist)
{
    unique_ptr<ToroboDriver> driver(createSingleArmGripper(7, "prismatic"));
    ASSERT_TRUE(driver->IsInit());

    // create publisher helper
    vector<string> controller_names {"arm_controller", "gripper_controller"};
    ControllerPublisherHelper pub_h(nh_, controller_names);

    // check connected subscribers num
    pub_h.assertExpectedGetNumSubscribers(1U);
}

TEST_F(ToroboDriverTestFixture, 7AxisSingleArmPrismaticGripperSrvsExist)
{
    unique_ptr<ToroboDriver> driver(createSingleArmGripper(7, "prismatic"));
    ASSERT_TRUE(driver->IsInit());

    // create service helper
    ControllerServiceHelper srv_h;

    // check service exists
    srv_h.assertWaitForService("arm_controller");
    srv_h.assertWaitForService("gripper_controller");
}

TEST_F(ToroboDriverTestFixture, 16AxisHumanoidSubsExist)
{
    unique_ptr<ToroboDriver> driver(createHumanoid(6, 6, 2, 2));
    ASSERT_TRUE(driver->IsInit());

    // create subscriber helper
    vector<string> controller_names {"left_arm_controller", "right_arm_controller", "torso_controller", "head_controller"};
    ControllerSubscriberHelper sub_h(nh_, controller_names);

    // check connected publishers num
    sub_h.assertExpectedGetNumPublishers(1U);

    // publish & subscribe
    driver->runOnce();
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // chech subscriber receive state
    sub_h.assertExpectedCallCount("left_arm_controller", 1U);
    sub_h.assertExpectedCallCount("right_arm_controller", 1U);
    sub_h.assertExpectedCallCount("torso_controller", 1U);
    sub_h.assertExpectedCallCount("head_controller", 1U);
    sub_h.assertExpectedReceiveState("left_arm_controller", 6U);
    sub_h.assertExpectedReceiveState("right_arm_controller", 6U);
    sub_h.assertExpectedReceiveState("torso_controller", 2U);
    sub_h.assertExpectedReceiveState("head_controller", 2U);
}

TEST_F(ToroboDriverTestFixture, 16AxisHumanoidPubsExist)
{
    unique_ptr<ToroboDriver> driver(createHumanoid(6, 6, 2, 2));
    ASSERT_TRUE(driver->IsInit());

    // create publisher helper
    vector<string> controller_names {"left_arm_controller", "right_arm_controller", "torso_controller", "head_controller"};
    ControllerPublisherHelper pub_h(nh_, controller_names);

    // check connected subscribers num
    pub_h.assertExpectedGetNumSubscribers(1U);
}

TEST_F(ToroboDriverTestFixture, 16AxisHumanoidSrvsExist)
{
    unique_ptr<ToroboDriver> driver(createHumanoid(6, 6, 2, 2));
    ASSERT_TRUE(driver->IsInit());

    // create service helper
    ControllerServiceHelper srv_h;

    // check service exists
    srv_h.assertWaitForService("left_arm_controller");
    srv_h.assertWaitForService("right_arm_controller");
    srv_h.assertWaitForService("torso_controller");
    srv_h.assertWaitForService("head_controller");
}

TEST_F(ToroboDriverTestFixture, 16AxisHumanoidRevoluteGripperSubsExist)
{
    unique_ptr<ToroboDriver> driver(createHumanoidGripper(6, 6, 2, 2, "revolute"));
    ASSERT_TRUE(driver->IsInit());

    // create subscriber helper
    vector<string> controller_names {"left_arm_controller", "right_arm_controller", "torso_controller", "head_controller",
         "left_gripper_controller", "right_gripper_controller"};
    ControllerSubscriberHelper sub_h(nh_, controller_names);

    // check connected publishers num
    sub_h.assertExpectedGetNumPublishers(1U);

    // publish & subscribe
    driver->runOnce();
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // chech subscriber receive state
    sub_h.assertExpectedCallCount("left_arm_controller", 1U);
    sub_h.assertExpectedCallCount("right_arm_controller", 1U);
    sub_h.assertExpectedCallCount("torso_controller", 1U);
    sub_h.assertExpectedCallCount("head_controller", 1U);
    sub_h.assertExpectedCallCount("left_gripper_controller", 1U);
    sub_h.assertExpectedCallCount("right_gripper_controller", 1U);
    sub_h.assertExpectedReceiveState("left_arm_controller", 6U);
    sub_h.assertExpectedReceiveState("right_arm_controller", 6U);
    sub_h.assertExpectedReceiveState("torso_controller", 2U);
    sub_h.assertExpectedReceiveState("head_controller", 2U);
    sub_h.assertExpectedReceiveState("left_gripper_controller", 1U);
    sub_h.assertExpectedReceiveState("right_gripper_controller", 1U);
}

TEST_F(ToroboDriverTestFixture, 16AxisHumanoidRevoluteGripperPubsExist)
{
    unique_ptr<ToroboDriver> driver(createHumanoidGripper(6, 6, 2, 2, "revolute"));
    ASSERT_TRUE(driver->IsInit());

    // create publisher helper
    vector<string> controller_names {"left_arm_controller", "right_arm_controller", "torso_controller", "head_controller",
         "left_gripper_controller", "right_gripper_controller"};
    ControllerPublisherHelper pub_h(nh_, controller_names);

    // check connected subscribers num
    pub_h.assertExpectedGetNumSubscribers(1U);
}

TEST_F(ToroboDriverTestFixture, 16AxisHumanoidRevoluteGripperSrvsExist)
{
    unique_ptr<ToroboDriver> driver(createHumanoidGripper(6, 6, 2, 2, "revolute"));
    ASSERT_TRUE(driver->IsInit());

    // create service helper
    ControllerServiceHelper srv_h;

    // check service exists
    srv_h.assertWaitForService("left_arm_controller");
    srv_h.assertWaitForService("right_arm_controller");
    srv_h.assertWaitForService("torso_controller");
    srv_h.assertWaitForService("head_controller");
    srv_h.assertWaitForService("left_gripper_controller");
    srv_h.assertWaitForService("right_gripper_controller");
}
