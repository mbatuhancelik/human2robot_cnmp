/**
 * @file  torobo_driver.h
 * @brief Torobo driver class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_DRIVER_H
#define TOROBO_DRIVER_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <memory>
#include "MasterControllerClient/MasterControllerClient.h"
#include "torobo_driver/torobo_controller_base.h"
#include "torobo_driver/torobo_driver_param.h"

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboDriver
{
public:
    ToroboDriver(ros::NodeHandle &nh,
        const ToroboDriverCommonParam& common_param,
        const ToroboDriverParam& param,
        const std::chrono::nanoseconds& period_ns,
        const double timeout_sec,
        bool is_mock = false,
        bool debug = false
    );
    virtual ~ToroboDriver();

    void run();
    void stop();
    void runOnce();

    bool Initialize();
    bool IsInit();

    void PrintParam();
    void SpinOnce();
    void Publish();
    void SendCommandToMaster();

protected:
    bool GetParamsFromRosParam(std::string configName, torobo::ToroboDriverParam& param);
    bool ConnectEthernet(std::string ip, int port);
    bool ConnectSerialPort(std::string comport, int baudrate);
    bool ConnectMock();

    bool init_;
    bool thread_work_;
    bool is_mock_;
    bool debug_;

    const std::chrono::nanoseconds period_ns_;

    ToroboDriverCommonParam common_param_;
    ToroboDriverParam param_;
    std::shared_ptr<MasterControllerClient> client_;
    std::vector<std::unique_ptr<ToroboControllerBase>> controllers_;

    double timeout_sec_;
    ros::Time last_recv_time_;

    ros::NodeHandle& nh_;
};

}

#endif
