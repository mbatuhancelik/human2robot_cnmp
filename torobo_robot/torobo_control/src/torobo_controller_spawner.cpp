/**
 * @file  torobo_controller_spawner.cpp
 * @brief ToroboControllerSpawner class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <string>
#include <map>
#include "torobo_control/torobo_joint_trajectory_controller.h"
#include "torobo_control/torobo_gripper_controller.h"
#include "torobo_control/torobo_joint_state_controller.h"
#include "torobo_control/torobo_controller_spawner.h"
#include <torobo_common/urdf_parser.h>


using namespace std;

namespace torobo
{

/*----------------------------------------------------------------------
 Public Method Implementations
 ----------------------------------------------------------------------*/
ToroboControllerSpawner::ToroboControllerSpawner(ros::NodeHandle &nh)
    : nh_(nh)
{
}

ToroboControllerSpawner::~ToroboControllerSpawner()
{
}


void ToroboControllerSpawner::run()
{
    std::string robot_description = "robot_description";

    // URDF model
    urdf::ModelSharedPtr urdf = torobo_common::UrdfParser::getUrdf(nh_, robot_description);
    if (!urdf) 
    {
        ROS_ERROR_STREAM("[torobo_control] error of loading rosparam : " << robot_description);
        return;
    }

    ROS_INFO("Start ToroboControllerSpawner");

    // Get controller param
    std::map<std::string, ToroboControllerSpawner::ControllerParam> controller_param = getControllerParam();

    // At first, create ToroboJointStateController
    bool joint_state_controller_is_created = false;
    for (const auto itr : controller_param)
    {
        auto name = itr.first;
        auto param = itr.second;

        if(param.type.find("JointStateController") != std::string::npos)
        {
            ROS_INFO_STREAM("[torobo_control] load controller: " << name);
            torobo_joint_state_controller_.reset(new ToroboJointStateController(nh_, "joint_state_controller"));
            torobo_joint_state_controller_->setPublishRate(param.publish_rate);
            torobo_joint_state_controller_->start();
            ROS_INFO_STREAM("  publish_rate : " << param.publish_rate);
            joint_state_controller_is_created = true;
        }
    }
    // Check if ToroboJointStateController was created.
    ROS_ASSERT(joint_state_controller_is_created);

    // Next, create ToroboActionControllers
    for (const auto itr : controller_param)
    {
        auto name = itr.first;
        auto param = itr.second;

        if(param.type.find("JointTrajectoryController") != std::string::npos)
        {
            ROS_INFO_STREAM("[torobo_control] load controller: " << name);
            unique_ptr<ToroboJointTrajectoryController> controller(new ToroboJointTrajectoryController(nh_, name, "follow_joint_trajectory"));

            // Create Joints and register them here.
            for(auto joint_name : param.joints)
            {
                urdf::JointConstSharedPtr urdf_joint = torobo_common::UrdfParser::getUrdfJoint(*urdf, joint_name);
                if (!urdf_joint)
                {
                    ROS_ERROR_STREAM("[torobo_control] urdf get UrdfJoint error : " << joint_name);
                    break;
                }
                try
                {
                    controller->registerJoint(urdf_joint->name, urdf_joint->limits->lower, urdf_joint->limits->upper, urdf_joint->limits->effort, urdf_joint->limits->velocity);
                }
                catch (...)
                {
                    ROS_ERROR_STREAM_NAMED("[torobo_control]", "reading urdf tag error");
                    break;
                }
            }

            controllers_.push_back(std::move(controller));
            torobo_joint_state_controller_->registerSourceTopic(name + "/joint_state");
        }
        else if(param.type.find("GripperActionController") != std::string::npos)
        {
            ROS_INFO_STREAM("[torobo_control_node] load controller: " << name);
            unique_ptr<ToroboGripperController> controller(new ToroboGripperController(nh_, name, "gripper_cmd"));

            std::string joint_name = param.joint;
            urdf::JointConstSharedPtr urdf_joint = torobo_common::UrdfParser::getUrdfJoint(*urdf, joint_name);
            if (!urdf_joint)
            {
                ROS_ERROR_STREAM("[torobo_control] urdf get UrdfJoint error : " << joint_name);
                break;
            }
            try
            {
                controller->registerJoint(urdf_joint->name, urdf_joint->limits->lower, urdf_joint->limits->upper, urdf_joint->limits->effort, urdf_joint->limits->velocity);
            }
            catch (...)
            {
                ROS_ERROR_STREAM_NAMED("[torobo_control]", "reading urdf tag error");
                break;
            }

            controllers_.push_back(std::move(controller));
            torobo_joint_state_controller_->registerSourceTopic(name + "/joint_state");
        }
    }
}

/*----------------------------------------------------------------------
 Protected Method Implementations
 ----------------------------------------------------------------------*/
std::map<std::string, ToroboControllerSpawner::ControllerParam> ToroboControllerSpawner::getControllerParam()
{
    std::vector<std::string> controller_list;
    nh_.getParam("controller_list", controller_list);

    std::map<std::string, ToroboControllerSpawner::ControllerParam> controller_param;

    for (auto name: controller_list)
    {
        ToroboControllerSpawner::ControllerParam param;
        nh_.param<std::string>(name + "/type", param.type, "");
        nh_.param<double>(name + "/publish_rate", param.publish_rate, 20.0);
        nh_.param<std::string>(name + "/joint", param.joint, "");
        nh_.param<std::vector<std::string> >(name + "/joints", param.joints, std::vector<std::string>());
        controller_param[name] = param;
    }

    return controller_param;    
}

} // namespace torobo
