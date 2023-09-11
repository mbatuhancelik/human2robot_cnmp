#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <gtest/gtest.h>
#include <memory>
#include <mutex>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommand.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <torobo_common/math_util.h>
#include <torobo_msgs/CancelTrajectory.h>
#include "torobo_control/torobo_joint_state_server.h"
#include "torobo_control/torobo_joint_state_controller.h"
#include "torobo_control/torobo_joint_trajectory_controller.h"
#include "torobo_control/torobo_gripper_controller.h"


using namespace std;
using namespace torobo;

int	main(int argc, char **argv)
{
    ros::init(argc, argv, "test_torobo_control_node");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

class ToroboControlTestFixture : public ::testing::Test
{
private:
    struct JointStatus
    {
        std::string joint_name_ = "";
        double joint_position_ = 0.0;
        double joint_velocity_ = 0.0;
        double start_position_ = 0.0;
        double target_position_ = 0.0;
        ros::Duration transition_time_ = ros::Duration(0.0); // commanded time
        ros::Time start_time_ = ros::Time(0.0);
        bool running_ = false;
        bool cancelled_ = false;
    };

    // member
    ros::NodeHandle nh_;
    ros::Subscriber sub_gripper_;
    ros::Publisher pub_joint_state_;
    ros::Timer timer_;
    ros::CallbackQueue callback_queue_;
    ros::ServiceServer server_cancel_;
    std::unique_ptr<ros::AsyncSpinner> async_spinner_ptr_;
    std::mutex mutex_;

    // controller members
    std::unique_ptr<ToroboJointStateController> torobo_joint_state_controller_;
    std::unique_ptr<ToroboGripperController> torobo_gripper_controller_;
    std::unique_ptr<ToroboJointStateServer> torobo_joint_state_server_;

public:
    // member
    JointStatus joint_status_;

    ToroboControlTestFixture()
    {
    }

    virtual void SetUp()
    {
        // Init Node Handle's callback queue and spinner
        nh_.setCallbackQueue(&callback_queue_);
        callback_queue_.callAvailable(ros::WallDuration());
        async_spinner_ptr_.reset(new ros::AsyncSpinner(1, &callback_queue_));
        async_spinner_ptr_->start();

        // Init members
        joint_status_.joint_name_ = "gripper/finger_joint";
        sub_gripper_ = nh_.subscribe("gripper_controller/command", 1, &ToroboControlTestFixture::gripperCommandCallback, this);
        pub_joint_state_ = nh_.advertise<torobo_msgs::ToroboJointState>("gripper_controller/torobo_joint_state", 1, this);
        timer_ = nh_.createTimer(ros::Duration(1.0 / 100.0), &ToroboControlTestFixture::timerCallback, this);

        // Create ToroboJointStateController
        ROS_INFO_STREAM("[torobo_control] load controller: " << "joint_state_controller");
        torobo_joint_state_controller_.reset(new ToroboJointStateController(nh_, "joint_state_controller"));
        torobo_joint_state_controller_->setPublishRate(20);
        torobo_joint_state_controller_->start();

        // Create ToroboGripperController
        std::string name = "gripper_controller";
        ROS_INFO_STREAM("[torobo_control] load controller: " << name);
        torobo_gripper_controller_.reset(new ToroboGripperController(nh_, name, "gripper_cmd"));
        torobo_gripper_controller_->registerJoint(joint_status_.joint_name_, 0.0, 0.08, 50.0, 0.5);
        torobo_joint_state_controller_->registerSourceTopic(name + "/joint_state");

        // Create ToroboJointStateServer
        ROS_INFO_STREAM("[torobo_control] load ToroboJointStateServer");
        torobo_joint_state_server_.reset(new ToroboJointStateServer(nh_));
        torobo_joint_state_server_->setPublishRate(10);
        torobo_joint_state_server_->registerSourceTopic("gripper_controller/torobo_joint_state");
        torobo_joint_state_server_->start();

        // Create TrajectoryCancelService
        ROS_INFO_STREAM("[torobo_control] create CancelTrajectoryService");
        server_cancel_ = nh_.advertiseService(name + "/cancel_trajectory", &ToroboControlTestFixture::cancelTrajectoryCallback, this);
    }

    virtual void TearDown()
    {
        timer_.stop(); // this stop is needed because Timer uses global callback_queue different with custom queue and its thread.
        sub_gripper_.shutdown();
        pub_joint_state_.shutdown();
        server_cancel_.shutdown();
        async_spinner_ptr_->stop();  // wait for finishing async spinner's thread
    }

    void timerCallback(const ros::TimerEvent& e)
    {
        // timer callback is done by global callback queue's thread

        std::lock_guard<std::mutex> lock(mutex_); // lock

        // Update joint_status_
        if(joint_status_.running_)
        {
            ros::Duration elapsed_time = ros::Time::now() - joint_status_.start_time_;
            if(elapsed_time > joint_status_.transition_time_)
            {
                joint_status_.joint_position_ = joint_status_.target_position_;
                joint_status_.joint_velocity_ = 0.0;
                joint_status_.running_ = false;
            }
            else
            {
                joint_status_.joint_position_ = (elapsed_time.toSec() / joint_status_.transition_time_.toSec()) * (joint_status_.target_position_ - joint_status_.start_position_) + joint_status_.start_position_;
                joint_status_.joint_velocity_ = (joint_status_.target_position_ - joint_status_.start_position_) / joint_status_.transition_time_.toSec();
            }
        }

        // Publish joint_state
        torobo_msgs::ToroboJointState::Ptr msg(new torobo_msgs::ToroboJointState());
        msg->header.stamp = ros::Time::now();
        msg->name.push_back(joint_status_.joint_name_);
        msg->position.push_back(joint_status_.joint_position_);
        msg->velocity.push_back(joint_status_.joint_velocity_);
        msg->effort.push_back(0.0);
        msg->trjStatus.push_back( (joint_status_.running_ ? ToroboJointTrajectoryController::TRAJ_STATUS_RUNNING : (joint_status_.cancelled_ ? ToroboJointTrajectoryController::TRAJ_STATUS_CANCEL_COMPLETE : ToroboJointTrajectoryController::TRAJ_STATUS_COMPLETE) ) ); // RUNNING
        pub_joint_state_.publish(msg);
    }

    void gripperCommandCallback(const control_msgs::GripperCommand::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_); // lock

        joint_status_.start_position_ = joint_status_.joint_position_;
        joint_status_.target_position_ = msg->position;
        joint_status_.transition_time_ = ros::Duration(5.0);
        joint_status_.start_time_ = ros::Time::now();
        joint_status_.running_ = true;
        joint_status_.cancelled_ = false;
    }

    bool cancelTrajectoryCallback(torobo_msgs::CancelTrajectory::Request &req, torobo_msgs::CancelTrajectory::Response &res)
    {
        std::lock_guard<std::mutex> lock(mutex_); // lock

        if(joint_status_.joint_name_ == req.joint_names[0])
        {
            ROS_INFO("cancel trajectory service is called");
            joint_status_.running_ = false;
            joint_status_.cancelled_ = true;
            res.success = true;
        }
        else
        {
            res.success = false;
        }
        return true;
    }

    void initJointStatus()
    {
        std::lock_guard<std::mutex> lock(mutex_); // lock

        joint_status_.joint_position_ = 0.0;
        joint_status_.joint_velocity_ = 0.0;
        joint_status_.start_position_ = 0.0;
        joint_status_.target_position_ = 0.0;
        joint_status_.transition_time_ = ros::Duration(0.0); // commanded time
        joint_status_.start_time_ = ros::Time(0.0);
        joint_status_.running_ = false;
        joint_status_.cancelled_ = false;

        ros::Duration(2.0).sleep();
    }
};


class ToroboControlActionClient
{
private:
    std::string action_name_ = "gripper_controller/gripper_cmd";
    std::unique_ptr<actionlib::SimpleActionClient<control_msgs::GripperCommandAction> > ac_;

public:
    ToroboControlActionClient()
    {
        ROS_INFO_STREAM("[torobo_control] Create SimpleActionClient");
        ac_.reset(new actionlib::SimpleActionClient<control_msgs::GripperCommandAction>(action_name_, true));
        ac_->waitForServer();
    }

    void publishGoal(double position, double max_effort)
    {
        ROS_INFO_STREAM("[torobo_control] Send goal");
        control_msgs::GripperCommandGoal goal;
        goal.command.position = position;
        goal.command.max_effort = max_effort;
        ac_->sendGoal(goal);
    }

    double waitForResult(double timeout)
    {
        ROS_INFO_STREAM("[torobo_control] Wait for result");
        ros::Time measure_start = ros::Time::now();
        bool finished_before_timeout = ac_->waitForResult(ros::Duration(timeout));
        ros::Time measure_end = ros::Time::now();
        double elapsed_time = (measure_end - measure_start).toSec();
        EXPECT_TRUE(finished_before_timeout);
        return elapsed_time;
    }

    actionlib::SimpleClientGoalState getState()
    {
        ROS_INFO_STREAM("[torobo_control] Check if action was successfully finished");
        actionlib::SimpleClientGoalState state = ac_->getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        return state;
    }

    control_msgs::GripperCommandResult::ConstPtr getResult()
    {
        ROS_INFO_STREAM("[torobo_control] Check result");
        return ac_->getResult();
    }
};


TEST_F(ToroboControlTestFixture, NormalTest_001_AllTest)
{

// TEST_F(ToroboControlTestFixture, NormalTest_001_BasicParameter) // If you want to divide test cases, it is needed to launch action server as an another node in launch file.
// {
    initJointStatus();
    {
        // test value
        double target_position = 0.08;
        double max_effort = 50.0;
        double action_timeout = 10.0;

        // create action client
        ToroboControlActionClient client;
        // publish goal
        client.publishGoal(target_position, max_effort);
        // wait for result
        double elapsed_time = client.waitForResult(action_timeout);

        // check state
        auto state = client.getState();
        EXPECT_EQ(state, actionlib::SimpleClientGoalState::SUCCEEDED);
        
        // check result
        auto result = client.getResult();
        EXPECT_FALSE(result->stalled);
        EXPECT_TRUE(result->reached_goal);

        // check position and elapsed time
        EXPECT_TRUE(math::nearly_equal(result->position, target_position, 0.001));
        EXPECT_TRUE((5.0 < elapsed_time && elapsed_time < 6.0));
    }
// }


// TEST_F(ToroboControlTestFixture, NormalTest_002_DoubleAction)
// {
    initJointStatus();
    {
        // test value
        double target_position_first = 0.08;
        double target_position_second = 0.02;
        double max_effort = 50.0;
        double action_timeout = 10.0;

        // create action client
        ToroboControlActionClient client;
        // publish goal
        client.publishGoal(target_position_first, max_effort);
        // wait for a while
        ros::Duration(2.0).sleep();
        // publish goal
        client.publishGoal(target_position_second, max_effort);
        // wait for result
        double elapsed_time = client.waitForResult(action_timeout);

        // check state
        auto state = client.getState();
        EXPECT_EQ(state, actionlib::SimpleClientGoalState::SUCCEEDED);
        
        // check result
        auto result = client.getResult();
        EXPECT_FALSE(result->stalled);
        EXPECT_TRUE(result->reached_goal);

        // check position and elapsed time
        EXPECT_TRUE(math::nearly_equal(result->position, target_position_second, 0.005));
        EXPECT_TRUE((5.0 < elapsed_time && elapsed_time < 6.0));
    }
// }

// TEST_F(ToroboControlTestFixture, ExceptionTest_001_PositionIsOverLimit)
// {
    initJointStatus();
    {
        // test value
        double target_position = 1.0; // over limit
        double max_effort = 50.0;
        double action_timeout = 10.0;

        // create action client
        ToroboControlActionClient client;
        // publish goal
        client.publishGoal(target_position, max_effort);
        // wait for result
        double elapsed_time = client.waitForResult(action_timeout);

        // check state
        auto state = client.getState();
        EXPECT_EQ(state, actionlib::SimpleClientGoalState::ABORTED);
        
        // check result
        auto result = client.getResult();
        EXPECT_FALSE(result->stalled);
        EXPECT_FALSE(result->reached_goal);

        // check position and elapsed time
        EXPECT_FALSE(math::nearly_equal(result->position, target_position, 0.001));
        EXPECT_TRUE((elapsed_time < 1.0));
    }
// }


// TEST_F(ToroboControlTestFixture, ExceptionTest_001_MaxeffortIsOverLimit)
// {
    initJointStatus();
    {
        // test value
        double target_position = 0.08;
        double max_effort = 100.0; // over limit
        double action_timeout = 10.0;

        // create action client
        ToroboControlActionClient client;
        // publish goal
        client.publishGoal(target_position, max_effort);
        // wait for result
        double elapsed_time = client.waitForResult(action_timeout);

        // check state
        auto state = client.getState();
        EXPECT_EQ(state, actionlib::SimpleClientGoalState::ABORTED);
        
        // check result
        auto result = client.getResult();
        EXPECT_FALSE(result->stalled);
        EXPECT_FALSE(result->reached_goal);

        // check position and elapsed time
        EXPECT_FALSE(math::nearly_equal(result->position, target_position, 0.001));
        EXPECT_TRUE((elapsed_time < 1.0));
    }
// }

}
