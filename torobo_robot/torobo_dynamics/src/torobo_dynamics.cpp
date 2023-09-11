/**
 * @file  ToroboDynamics.h
 * @brief Torobo Dynamics class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <vector>
#include <regex>
#include "torobo_dynamics/torobo_dynamics.h"
#include "torobo_dynamics/joint_state.h"

using namespace std;

namespace torobo
{
/*----------------------------------------------------------------------
 Privat Global Variables
 ----------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
ToroboDynamics::ToroboDynamics(ros::NodeHandle& node, const std::map<std::string, std::vector<std::string>>& controller_joints_map, bool sim)
 : node_(node), controller_joints_map_(controller_joints_map), sim_(sim)
{
    model_.reset(new ToroboRbdlModel());

    for(auto itr = controller_joints_map_.begin(); itr != controller_joints_map_.end(); ++itr)
    {
        const string& name = itr->first;

        // excluding gripper_controller
        if(name.find("gripper") != std::string::npos)
        {
            continue;
        }

        // excluding arm_controller not included in humanoid in real mode
        if(name.find("arm_controller") != std::string::npos)
        {
            if(!sim && (name != "left_arm_controller" && name != "right_arm_controller"))
            {
                continue;
            }
        }

        // Generate dynamics publisher
        ros::Publisher pub  = node_.advertise<torobo_msgs::ToroboDynamics>(name + "/torobo_dynamics", 1);
        torobo_msgs::ToroboDynamics dyna;
        InitializeToroboDynamics(dyna, itr->second);
        dynamics_.insert(make_pair(name, dyna));
        torobo_dynamics_pub_.insert(make_pair(name, pub));

        //Generate ServiceServer
        if(name == "left_arm_controller" || name == "right_arm_controller")
        {
            ros::ServiceServer srv = node_.advertiseService(name + "/set_payload_param", &ToroboDynamics::SetPayloadParamService,this);
            service_.insert(make_pair(name, srv));
        }
    }
}

ToroboDynamics::~ToroboDynamics()
{
}

void ToroboDynamics::Update(const std::unique_ptr<ToroboState>& toroboState)
{
    // Update gravity compensation effort
    for(auto itr = controller_joints_map_.begin(); itr != controller_joints_map_.end(); ++itr)
    {
        const string& prefix = itr->first;
        const JointState* const jointState = toroboState->GetToroboJointState(prefix);

        // Calc gravity compensation effort
        const vector<double> zeroVec(jointState->joint_names_.size(), 0.0);
        model_->UpdateQ(    jointState->joint_names_, jointState->position_);
        model_->UpdateQDot( jointState->joint_names_, zeroVec);
        model_->UpdateQDDot(jointState->joint_names_, zeroVec);
    }
    model_->CalcInverseDynamics();
    for(auto itr = controller_joints_map_.begin(); itr != controller_joints_map_.end(); ++itr)
    {
        const string& prefix = itr->first;
        SetGravityCompensationEffort(prefix, model_->GetTau(itr->second));
    }

    // Update cur dynamics effort & inertia matrix
    for(auto itr = controller_joints_map_.begin(); itr != controller_joints_map_.end(); ++itr)
    {
        const string& prefix = itr->first;
        const JointState* const jointState = toroboState->GetToroboJointState(prefix);

        // Calc gravity compensation effort
        model_->UpdateQDot( jointState->joint_names_, jointState->velocity_);
        model_->UpdateQDDot(jointState->joint_names_, jointState->acceleration_);
    }
    model_->CalcInverseDynamics();
    model_->CalcCompositeRigidBodyAlgorithm();
    for(auto itr = controller_joints_map_.begin(); itr != controller_joints_map_.end(); ++itr)
    {
        const string& prefix = itr->first;
        SetCurDynamicsEffort(prefix, model_->GetTau(controller_joints_map_[prefix]));
        SetInertiaDiagonal(prefix, model_->GetInertiaDiagonal(controller_joints_map_[prefix]));
    }

    // Update ref dynamics effort
    for(auto itr = controller_joints_map_.begin(); itr != controller_joints_map_.end(); ++itr)
    {
        const string& prefix = itr->first;
        const JointState* const jointState = toroboState->GetToroboJointState(prefix);

        // Calc gravity compensation effort
        model_->UpdateQDot( jointState->joint_names_, jointState->ref_velocity_);
        model_->UpdateQDDot(jointState->joint_names_, jointState->ref_acceleration_);
    }
    model_->CalcInverseDynamics();
    for(auto itr = controller_joints_map_.begin(); itr != controller_joints_map_.end(); ++itr)
    {
        const string& prefix = itr->first;
        SetRefDynamicsEffort(prefix, model_->GetTau(controller_joints_map_[prefix]));
    }
}

void ToroboDynamics::Publish()
{
    for(auto itr = dynamics_.begin(); itr != dynamics_.end(); ++itr)
    {
        string name = itr->first;
        if(torobo_dynamics_pub_.count(name) == 0)
        {
            continue;
        }
        itr->second.header.stamp = ros::Time::now();
        torobo_dynamics_pub_.at(name).publish(itr->second);
    }
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
void ToroboDynamics::InitializeToroboDynamics(torobo_msgs::ToroboDynamics& dyna, const std::vector<std::string>& joint_names) const
{
    // Initialize parameters
    dyna.header = std_msgs::Header();
    dyna.header.stamp = ros::Time::now();
    for(int i = 0; i < joint_names.size(); i++)
    {
        dyna.name.push_back(joint_names[i]);
        dyna.gravity_compensation_effort.push_back(0.0);
        dyna.ref_dynamics_effort.push_back(0.0);
        dyna.cur_dynamics_effort.push_back(0.0);
        dyna.inertia_diagonal.push_back(0.0);
    }
}

void ToroboDynamics::SetRefDynamicsEffort(const std::string& name, const std::vector<double>& effort)
{
    size_t s = dynamics_[name].ref_dynamics_effort.size();
    if(s != effort.size())
    {
        return;
    }
    for(int i = 0; i < s; i++)
    {
        dynamics_[name].ref_dynamics_effort[i] = effort[i];
    }
}

void ToroboDynamics::SetCurDynamicsEffort(const std::string& name, const std::vector<double>& effort)
{
    size_t s = dynamics_[name].cur_dynamics_effort.size();
    if(s != effort.size())
    {
        return;
    }
    for(int i = 0; i < s; i++)
    {
        dynamics_[name].cur_dynamics_effort[i] = effort[i];
    }
}

void ToroboDynamics::SetInertiaDiagonal(const std::string& name, const std::vector<double>& inertiaDiagonal)
{
    size_t s = dynamics_[name].inertia_diagonal.size();
    if(s != inertiaDiagonal.size())
    {
        return;
    }
    for(int i = 0; i < s; i++)
    {
        dynamics_[name].inertia_diagonal[i] = inertiaDiagonal[i];
    }
}

void ToroboDynamics::SetGravityCompensationEffort(const std::string& name, const std::vector<double>& gravityCompEffort)
{
    size_t s = dynamics_[name].gravity_compensation_effort.size();
    if(s != gravityCompEffort.size())
    {
        return;
    }
    for(int i = 0; i < s; i++)
    {
        dynamics_[name].gravity_compensation_effort[i] = gravityCompEffort[i];
    }
}

bool ToroboDynamics::SetPayloadParamService(torobo_msgs::SetPayloadParam::Request &req , torobo_msgs::SetPayloadParam::Response &res)
{
    ROS_INFO("[ToroboDynamics:%s] SetPayloadParamService is called.", req.name.c_str());

    if(controller_joints_map_.count(req.name) == 0)
    {
        ROS_ERROR("[ToroboDynamics:%s] Invalid name is given. Failed to add fixed body.", req.name.c_str());
        res.success = false;
        return false;
    }
    const int jointsNum = controller_joints_map_.at(req.name).size();

    RigidBodyDynamics::Math::Vector3d com(0.0, 0.0, 0.0);
    if(req.com.size() == 3)
    {
        com = RigidBodyDynamics::Math::Vector3d(req.com[0], req.com[1], req.com[2]);
    }
    
    double mass = req.mass;
    RigidBodyDynamics::Math::Matrix3d inertiaMatrix;
    double defaultMomentOfInertia = 1.0e-8;

    if(req.inertiaElem.size() == 6) 
    {
    //    ROS_INFO("ixx:%f, ixy:%f, ixz:%f, iyy:%f, iyz:%f, izz:%f is set.",req.inertiaElem[0],req.inertiaElem[1],req.inertiaElem[2],req.inertiaElem[3],req.inertiaElem[4],req.inertiaElem[5]);
        inertiaMatrix << req.inertiaElem[0] , req.inertiaElem[1] , req.inertiaElem[2] ,
                         req.inertiaElem[1] , req.inertiaElem[3] , req.inertiaElem[4] , 
                         req.inertiaElem[2] , req.inertiaElem[4] , req.inertiaElem[5] ;
    }
    else
    {
    //    ROS_INFO("ixx:%f, ixy:%f, ixz:%f, iyy:%f, iyz:%f, izz:%f is set.",defaultMomentOfInertia, 0.0, 0.0, defaultMomentOfInertia, 0.0, defaultMomentOfInertia);
        inertiaMatrix << defaultMomentOfInertia, 0.0  , 0.0,
                         0.0    ,defaultMomentOfInertia, 0.0,
                         0.0    ,   0.0, defaultMomentOfInertia;
    }

    const std::string prefix = std::regex_replace(req.name, std::regex("(.*)_controller"), "$1");
    res.success = model_->AddFixedBody(prefix, jointsNum, mass , com,inertiaMatrix);
    if(!res.success)
    {
        ROS_ERROR("[ToroboDynamics:%s] Failed to add fixed body.", req.name.c_str());
        return false;
    }
    ROS_INFO("[ToroboDynamics:%s] Succeeded to add fixed body.", req.name.c_str());
    return true;
}

}
