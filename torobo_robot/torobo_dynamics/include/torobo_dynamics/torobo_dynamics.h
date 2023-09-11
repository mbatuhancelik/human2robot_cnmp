/**
 * @file  ToroboDynamics.h
 * @brief Torobo Dynamics class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_DYNAMICS_H
#define TOROBO_DYNAMICS_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <string>
#include <map>
#include <vector>
#include <memory>
#include "torobo_dynamics/torobo_state.h"
#include "torobo_dynamics/torobo_rbdl_model.h"
#include "torobo_msgs/SetPayloadParam.h"

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboDynamics
{
public:
    ToroboDynamics(ros::NodeHandle& node, const std::map<std::string, std::vector<std::string>>& controller_joints_map, bool sim);
    virtual ~ToroboDynamics();

    void Update(const std::unique_ptr<ToroboState>& toroboState);
    void Publish();

protected:
    void InitializeToroboDynamics(torobo_msgs::ToroboDynamics& dyna, const std::vector<std::string>& joint_names) const;
    void SetRefDynamicsEffort(const std::string& name, const std::vector<double>& effort);
    void SetCurDynamicsEffort(const std::string& name, const std::vector<double>& effort);
    void SetInertiaDiagonal(const std::string& name, const std::vector<double>& diagonalElem);
    void SetGravityCompensationEffort(const std::string& name, const std::vector<double>& gravityCompEffort);
    bool SetPayloadParamService(torobo_msgs::SetPayloadParam::Request &req , torobo_msgs::SetPayloadParam::Response &res);

    std::unique_ptr<ToroboRbdlModel> model_;
    bool sim_;

    ros::NodeHandle& node_;
    std::map<std::string, ros::Publisher> torobo_dynamics_pub_;
    std::map<std::string, ros::ServiceServer> service_;

    std::map<std::string, std::vector<std::string>> controller_joints_map_;
    std::map<std::string, torobo_msgs::ToroboDynamics> dynamics_;
private:
};

}

#endif
