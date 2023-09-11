#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <gtest/gtest.h>
#include <memory>
#include <mutex>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <torobo_common/math_util.h>
#include <torobo_msgs/ToroboJointState.h>
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
    ros::Subscriber sub_trajectory_;
    ros::Publisher pub_joint_state_;
    ros::Timer timer_;
    ros::CallbackQueue callback_queue_;
    ros::ServiceServer server_cancel_;
    std::unique_ptr<ros::AsyncSpinner> async_spinner_ptr_;
    std::mutex mutex_;

    // controller members
    std::unique_ptr<ToroboJointStateController> torobo_joint_state_controller_;
    std::unique_ptr<ToroboJointTrajectoryController> torobo_joint_trajectory_controller_;
    std::unique_ptr<ToroboJointStateServer> torobo_joint_state_server_;

    // additional movement and time for test
    double additional_movement_ = 0.0;
    ros::Duration additional_transition_time_ = ros::Duration(0.0);

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
        joint_status_.joint_name_ = "arm/joint_1";
        sub_trajectory_ = nh_.subscribe("arm_controller/command", 1, &ToroboControlTestFixture::trajectoryCommandCallback, this);
        pub_joint_state_ = nh_.advertise<torobo_msgs::ToroboJointState>("arm_controller/torobo_joint_state", 1, this);
        timer_ = nh_.createTimer(ros::Duration(1.0 / 100.0), &ToroboControlTestFixture::timerCallback, this);

        // Create ToroboJointStateController
        ROS_INFO_STREAM("[torobo_control] load controller: " << "joint_state_controller");
        torobo_joint_state_controller_.reset(new ToroboJointStateController(nh_, "joint_state_controller"));
        torobo_joint_state_controller_->setPublishRate(20);
        torobo_joint_state_controller_->start();

        // Create ToroboJointTrajectoryController
        std::string name = "arm_controller";
        ROS_INFO_STREAM("[torobo_control] load controller: " << name);
        torobo_joint_trajectory_controller_.reset(new ToroboJointTrajectoryController(nh_, name, "follow_joint_trajectory"));
        torobo_joint_trajectory_controller_->registerJoint(joint_status_.joint_name_, -2.0, 2.0, 100.0, 10.0);
        torobo_joint_state_controller_->registerSourceTopic(name + "/joint_state");

        // Create ToroboJointStateServer
        ROS_INFO_STREAM("[torobo_control] load ToroboJointStateServer");
        torobo_joint_state_server_.reset(new ToroboJointStateServer(nh_));
        torobo_joint_state_server_->setPublishRate(10);
        torobo_joint_state_server_->registerSourceTopic("arm_controller/torobo_joint_state");
        torobo_joint_state_server_->start();

        // Create TrajectoryCancelService
        ROS_INFO_STREAM("[torobo_control] create CancelTrajectoryService");
        server_cancel_ = nh_.advertiseService(name + "/cancel_trajectory", &ToroboControlTestFixture::cancelTrajectoryCallback, this);
    }

    virtual void TearDown()
    {
        timer_.stop(); // this stop is needed because Timer uses global callback_queue different with custom queue and its thread.
        sub_trajectory_.shutdown();
        pub_joint_state_.shutdown();
        server_cancel_.shutdown();
        async_spinner_ptr_->stop();  // wait for finishing async spinner's thread
    }

    void timerCallback(const ros::TimerEvent& e)
    {
        std::lock_guard<std::mutex> lock(mutex_); // lock

        // update joint_status_
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

        // publish joint_state
        torobo_msgs::ToroboJointState::Ptr msg(new torobo_msgs::ToroboJointState());
        msg->header.stamp = ros::Time::now();
        msg->name.push_back(joint_status_.joint_name_);
        msg->position.push_back(joint_status_.joint_position_);
        msg->velocity.push_back(joint_status_.joint_velocity_);
        msg->effort.push_back(0.0);
        msg->trjStatus.push_back( (joint_status_.running_ ? ToroboJointTrajectoryController::TRAJ_STATUS_RUNNING : (joint_status_.cancelled_ ? ToroboJointTrajectoryController::TRAJ_STATUS_CANCEL_COMPLETE : ToroboJointTrajectoryController::TRAJ_STATUS_COMPLETE) ) ); // RUNNING
        pub_joint_state_.publish(msg);
    }

    void trajectoryCommandCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_); // lock

        if(msg->joint_names[0] == joint_status_.joint_name_)
        {
            joint_status_.start_position_ = joint_status_.joint_position_;
            joint_status_.target_position_ = msg->points[0].positions[0] + additional_movement_;
            joint_status_.transition_time_ = msg->points[0].time_from_start + additional_transition_time_;
            joint_status_.start_time_ = ros::Time::now();
            joint_status_.running_ = true;
            joint_status_.cancelled_ = false;
        }
        else
        {
            ROS_ERROR("JointTrajectory's joint_name [%s] does not match to trajectory_controller's joint_name", msg->joint_names[0].c_str());
        }
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

            // // publish joint_state
            // torobo_msgs::ToroboJointState::Ptr msg(new torobo_msgs::ToroboJointState());
            // msg->header.stamp = ros::Time::now();
            // msg->name.push_back(joint_status_.joint_name_);
            // msg->position.push_back(joint_status_.joint_position_);
            // msg->velocity.push_back(joint_status_.joint_velocity_);
            // msg->effort.push_back(0.0);
            // msg->trjStatus.push_back( ToroboJointTrajectoryController::TRAJ_STATUS_CANCEL_COMPLETE ); // RUNNING
            // pub_joint_state_.publish(msg);

        }
        else
        {
            res.success = false;
        }
        return true;
    }

    void setAdditionalMovement(double value)
    {
        std::lock_guard<std::mutex> lock(mutex_); // lock

        additional_movement_ = value;
    }

    void setAdditionalTransitionTime(double value)
    {
        std::lock_guard<std::mutex> lock(mutex_); // lock

        additional_transition_time_ = ros::Duration(value);
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

        additional_movement_ = 0.0;
        additional_transition_time_ = ros::Duration(0.0);

        ros::Duration(2.0).sleep();
    }
};


class ToroboControlActionClient
{
private:
    std::string action_name_ = "arm_controller/follow_joint_trajectory";
    std::unique_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> > ac_;

public:
    ToroboControlActionClient()
    {
        ROS_INFO_STREAM("[torobo_control] Create SimpleActionClient");
        ac_.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(action_name_, true));
        ac_->waitForServer();
    }

    void publishGoal(std::string joint_name, double target_position, double transition_time, double goal_tolerance=0.0, double goal_time_tolerance=0.0)
    {
        ROS_INFO_STREAM("[torobo_control] Send goal");
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.header.stamp = ros::Time::now();
        goal.trajectory.joint_names.push_back(joint_name);
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(target_position);
        point.velocities.push_back(0.0);
        point.accelerations.push_back(0.0);
        point.time_from_start = ros::Duration(transition_time);
        goal.trajectory.points.push_back(point);
        control_msgs::JointTolerance tolerance;
        tolerance.name = joint_name;
        tolerance.position = goal_tolerance;
        goal.goal_tolerance.push_back(tolerance);
        goal.goal_time_tolerance = ros::Duration(goal_time_tolerance);
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

    control_msgs::FollowJointTrajectoryResult::ConstPtr getResult()
    {
        ROS_INFO_STREAM("[torobo_control] Check result");
        return ac_->getResult();
    }
};


TEST_F(ToroboControlTestFixture, NormalTest_001_AllTest)
{

// TEST_F(ToroboControlTestFixture, NormalTest_001_BasicParameter)
// {
    initJointStatus();
    {
        // test value
        double target_position = 1.5;
        double transition_time = 5.0;
        double goal_tolerance = 0.0;
        double goal_time_tolerance = 0.0;
        double action_timeout = transition_time + 5.0;

        // create action client
        ToroboControlActionClient client;
        // publish goal
        client.publishGoal(joint_status_.joint_name_, target_position, transition_time, goal_tolerance, goal_time_tolerance);
        // wait for result
        double elapsed_time = client.waitForResult(action_timeout);

        // check state
        auto state = client.getState();
        EXPECT_EQ(state, actionlib::SimpleClientGoalState::SUCCEEDED);

        // check result
        auto result = client.getResult();
        EXPECT_EQ(result->error_code, control_msgs::FollowJointTrajectoryResult::SUCCESSFUL);

        // check position and elapsed time
        EXPECT_TRUE(math::nearly_equal(joint_status_.joint_position_, target_position, 0.01));
        EXPECT_TRUE((transition_time < elapsed_time && elapsed_time < transition_time + 1.0));
    }
// }


// TEST_F(ToroboControlTestFixture, NormalTest_002_CheckForGoalTolerance)
// {
    initJointStatus();
    {
        // test value
        double target_position = 1.5;
        double transition_time = 5.0;
        double goal_tolerance = 0.2; // set tolerance
        double goal_time_tolerance = 0.0;
        double action_timeout = transition_time + 5.0;
        setAdditionalMovement(0.1); // add noise in goal_tolerance

        // create action client
        ToroboControlActionClient client;
        // publish goal
        client.publishGoal(joint_status_.joint_name_, target_position, transition_time, goal_tolerance, goal_time_tolerance);
        // wait for result
        double elapsed_time = client.waitForResult(action_timeout);

        // check state
        auto state = client.getState();
        EXPECT_EQ(state, actionlib::SimpleClientGoalState::SUCCEEDED);

        // check result
        auto result = client.getResult();
        EXPECT_EQ(result->error_code, control_msgs::FollowJointTrajectoryResult::SUCCESSFUL);

        // check position and elapsed time
        EXPECT_FALSE(math::nearly_equal(joint_status_.joint_position_, target_position, 0.01));
        EXPECT_TRUE((transition_time < elapsed_time && elapsed_time < transition_time + 1.0));
    }
// }


// TEST_F(ToroboControlTestFixture, ExceptionTest_001_CheckForGoalTolerance)
// {
    initJointStatus();
    {
        // test value
        double target_position = 1.5;
        double transition_time = 5.0;
        double goal_tolerance = 0.0;
        double goal_time_tolerance = 0.0;
        double action_timeout = transition_time + 5.0;
        setAdditionalMovement(0.1); // add noise over goal_tolerance

        // create action client
        ToroboControlActionClient client;
        // publish goal
        client.publishGoal(joint_status_.joint_name_, target_position, transition_time, goal_tolerance, goal_time_tolerance);
        // wait for result
        double elapsed_time = client.waitForResult(action_timeout);

        // check state
        auto state = client.getState();
        EXPECT_EQ(state, actionlib::SimpleClientGoalState::ABORTED);

        // check result
        auto result = client.getResult();
        EXPECT_EQ(result->error_code, control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED); // ERROR

        // check position and elapsed time
        EXPECT_FALSE(math::nearly_equal(joint_status_.joint_position_, target_position, 0.01));
        EXPECT_TRUE((transition_time < elapsed_time && elapsed_time < transition_time + 1.0));
    }
// }


// TEST_F(ToroboControlTestFixture, NormalTest_003_CheckForGoalTimeTolerance)
// {
    initJointStatus();
    {
        // test value
        double target_position = 1.5;
        double transition_time = 5.0;
        double goal_tolerance = 0.0;
        double goal_time_tolerance = 2.0; // set tolerance
        double action_timeout = transition_time + 5.0;
        setAdditionalTransitionTime(1.0); // add noise in goal_time tolerance

        // create action client
        ToroboControlActionClient client;
        // publish goal
        client.publishGoal(joint_status_.joint_name_, target_position, transition_time, goal_tolerance, goal_time_tolerance);
        // wait for result
        double elapsed_time = client.waitForResult(action_timeout);

        // check state
        auto state = client.getState();
        EXPECT_EQ(state, actionlib::SimpleClientGoalState::SUCCEEDED);

        // check result
        auto result = client.getResult();
        EXPECT_EQ(result->error_code, control_msgs::FollowJointTrajectoryResult::SUCCESSFUL);

        // check position and elapsed time
        EXPECT_TRUE(math::nearly_equal(joint_status_.joint_position_, target_position, 0.01));
        EXPECT_TRUE((transition_time < elapsed_time && elapsed_time < transition_time + 3.0));
    }
// }


// TEST_F(ToroboControlTestFixture, ExceptionTest_002_CheckForGoalTimeTolerance)
// {
    initJointStatus();
    {
        // test value
        double target_position = 1.5;
        double transition_time = 5.0;
        double goal_tolerance = 0.0;
        double goal_time_tolerance = 0.0;
        double action_timeout = transition_time + 5.0;
        setAdditionalTransitionTime(2.0); // add noise over goal_time tolerance

        // create action client
        ToroboControlActionClient client;
        // publish goal
        client.publishGoal(joint_status_.joint_name_, target_position, transition_time, goal_tolerance, goal_time_tolerance);
        // wait for result
        double elapsed_time = client.waitForResult(action_timeout);

        // check state
        auto state = client.getState();
        EXPECT_EQ(state, actionlib::SimpleClientGoalState::ABORTED);

        // check result
        auto result = client.getResult();
        EXPECT_EQ(result->error_code, control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED); // ERROR

        // check position and elapsed time
        EXPECT_TRUE(math::nearly_equal(joint_status_.joint_position_, target_position, 0.01));
        EXPECT_FALSE((transition_time < elapsed_time && elapsed_time < transition_time + 1.0));
    }
// }


// TEST_F(ToroboControlTestFixture, ExceptionTest_003_InvalidJointName)
// {
    initJointStatus();
    {
        // test value
        double target_position = 1.5;
        double transition_time = 5.0;
        double goal_tolerance = 0.0;
        double goal_time_tolerance = 0.0;
        double action_timeout = transition_time + 5.0;

        // create action client
        ToroboControlActionClient client;
        // publish goal
        client.publishGoal("invalid_joint_name", target_position, transition_time); // invalid joint name
        // wait for result
        double elapsed_time = client.waitForResult(action_timeout);

        // check state
        auto state = client.getState();
        EXPECT_EQ(state, actionlib::SimpleClientGoalState::ABORTED);

        // check result
        auto result = client.getResult();
        EXPECT_EQ(result->error_code, control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS); // ERROR

        // check position and elapsed time
        EXPECT_FALSE(math::nearly_equal(joint_status_.joint_position_, target_position, 0.01));
        EXPECT_TRUE((elapsed_time < 1.0));
    }
// }


// TEST_F(ToroboControlTestFixture, ExceptionTest_003_InvalidGoalPosition)
// {
    initJointStatus();
    {
        // test value
        double target_position = 10.0; // invalid
        double transition_time = 5.0;
        double goal_tolerance = 0.0;
        double goal_time_tolerance = 0.0;
        double action_timeout = transition_time + 5.0;

        // create action client
        ToroboControlActionClient client;
        // publish goal
        client.publishGoal(joint_status_.joint_name_, target_position, transition_time, goal_tolerance, goal_time_tolerance);
        // wait for result
        double elapsed_time = client.waitForResult(action_timeout);

        // check state
        auto state = client.getState();
        EXPECT_EQ(state, actionlib::SimpleClientGoalState::ABORTED);

        // check result
        auto result = client.getResult();
        EXPECT_EQ(result->error_code, control_msgs::FollowJointTrajectoryResult::INVALID_GOAL); // ERROR

        // check position and elapsed time
        EXPECT_FALSE(math::nearly_equal(joint_status_.joint_position_, target_position, 0.01));
        EXPECT_TRUE((elapsed_time < 1.0));
    }
// }


// TEST_F(ToroboControlTestFixture, NormalTest_004_DoubleAction)
// {
    initJointStatus();
    {
        // test value
        double target_position = 1.5;
        double transition_time = 10.0;
        double goal_tolerance = 0.0;
        double goal_time_tolerance = 0.0;
        double action_timeout = transition_time + 5.0;

        // create action client
        ToroboControlActionClient client;
        // publish goal
        client.publishGoal(joint_status_.joint_name_, -target_position, transition_time, goal_tolerance, goal_time_tolerance); // first action
        // wait for a while
        ros::Duration(2.0).sleep();
        // publish goal
        client.publishGoal(joint_status_.joint_name_, target_position, transition_time, goal_tolerance, goal_time_tolerance); // second action
        // wait for result
        double elapsed_time = client.waitForResult(action_timeout);

        // check state
        auto state = client.getState();
        EXPECT_EQ(state, actionlib::SimpleClientGoalState::SUCCEEDED);

        // check result
        auto result = client.getResult();
        EXPECT_EQ(result->error_code, control_msgs::FollowJointTrajectoryResult::SUCCESSFUL); // ERROR

        // check position and elapsed time
        EXPECT_TRUE(math::nearly_equal(joint_status_.joint_position_, target_position, 0.01));
        EXPECT_TRUE((transition_time < elapsed_time && elapsed_time < transition_time + 1.0));
    }
// }


// TEST_F(ToroboControlTestFixture, NormalTest_005_SameTargetPositionAsCurrentPosition)
// {
    initJointStatus();
    {
        // test value
        double target_position = 0.0; // same as init position
        double transition_time = 5.0;
        double goal_tolerance = 0.0;
        double goal_time_tolerance = 0.0;
        double action_timeout = transition_time + 5.0;

        EXPECT_TRUE(math::nearly_equal(joint_status_.joint_position_, 0.0, 0.001));

        // create action client
        ToroboControlActionClient client;
        // publish goal
        client.publishGoal(joint_status_.joint_name_, target_position, transition_time, goal_tolerance, goal_time_tolerance);
        // wait for result
        double elapsed_time = client.waitForResult(action_timeout);

        // check state and result
        auto state = client.getState();
        EXPECT_EQ(state, actionlib::SimpleClientGoalState::SUCCEEDED);

        // check result
        auto result = client.getResult();
        EXPECT_EQ(result->error_code, control_msgs::FollowJointTrajectoryResult::SUCCESSFUL); // ERROR

        // check position and elapsed time
        EXPECT_TRUE(math::nearly_equal(joint_status_.joint_position_, target_position, 0.01));
        EXPECT_TRUE((transition_time < elapsed_time && elapsed_time < transition_time + 1.0));
    }
// }

}
