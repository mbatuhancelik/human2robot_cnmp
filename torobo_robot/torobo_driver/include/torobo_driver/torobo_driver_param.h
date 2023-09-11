/**
 * @file  torobo_driver_param.h
 * @brief Torobo driver parameter class
 *
 * @par   Copyright © 2019 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_DRIVER_PARAM_H
#define TOROBO_DRIVER_PARAM_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <vector>
#include <string>

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ControllerParam
{
public:
    ControllerParam(){}
    ~ControllerParam(){}
    std::vector<std::string> joint_names;
    std::vector<int> joint_ids;
    std::vector<std::string> joint_types;
};

class ToroboDriverParam
{
public:
    ToroboDriverParam()
    {
        controller.clear();
        allJointsNum = 0;
        interface = "";
        port = 0;
        com = "";
        baudrate = 0;
    }
    virtual ~ToroboDriverParam(){}

    std::vector<std::pair<std::string, ControllerParam>> controller;
    int allJointsNum;
    std::string interface;
    std::string ip;
    int port;
    std::string com;
    int baudrate;
};

class ToroboDriverCommonParam
{
public:
    ToroboDriverCommonParam(){}
    ~ToroboDriverCommonParam(){}
    double allowed_start_tolerance;
};

}

#endif
