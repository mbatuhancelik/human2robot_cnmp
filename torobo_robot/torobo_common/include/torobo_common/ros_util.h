/**
 * @file  ros_util.h
 * @brief ROS utilities
 *
 * @par   Copyright © 2019 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __ROS_UTIL_H__
#define __ROS_UTIL_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <ros/service_client.h>

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
namespace torobo_common
{

template <typename T>
bool waitForServiceWithRetry(
    ros::ServiceClient& out_srv,
    ros::NodeHandle &nh, const std::string &service_ns,
    const int max_retry_num, ros::Duration timeout = ros::Duration(3.0))
{
    out_srv = nh.serviceClient<T>(service_ns);
    for (int retry = 0; retry < max_retry_num; retry++)
    {
        if (retry > 0)
        {
            ROS_WARN_STREAM("Wait for service [" << service_ns << "] is timed out. Retry [" << retry << "/" << max_retry_num << "]...");
        }
        if (out_srv.waitForExistence(timeout))
        {
            return true;
        }
    }
    ROS_ERROR_STREAM("Failed to wait for service " << service_ns << "with " << max_retry_num << " retries.");
    return false;
}

} // namespace torobo_common

#endif
