/**
 * @file  torobo_driver_nodecore.h
 * @brief torobo driver nodecore class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_DRIVER_NODECORE_H
#define TOROBO_DRIVER_NODECORE_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <memory.h>
#include <vector>
#include <thread>
#include "torobo_driver/torobo_driver.h"
#include "torobo_driver/torobo_driver_param.h"

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboDriverNodeCore
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::map<std::string, std::unique_ptr<torobo::ToroboDriver>> drivers_;

    bool debug_;
    bool mock_;
    double rate_;
    double timeout_sec_;
    std::string robot_description_name_;
    std::vector<std::unique_ptr<std::thread>> threads_;

public:
    ToroboDriverNodeCore(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~ToroboDriverNodeCore();

    void run();
    void runOnce();

    double getRate()
    {
        return rate_;
    }

protected:
    bool GetControllerParamsFromRosParam(const std::string& config_name, const std::string& robot_description_param_name, std::vector<std::pair<std::string, torobo::ToroboDriverParam>>& param_vec);
    bool GetSignleControllerParamFromRosParam(const std::string& controller_config_name, torobo::ToroboDriverParam& param);
    void PrintParam(const std::vector<std::pair<std::string,torobo::ToroboDriverParam>>& param_vec);
};

}

#endif
