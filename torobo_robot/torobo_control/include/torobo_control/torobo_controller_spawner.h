/**
 * @file  torobo_controller_spawner.h
 * @brief ToroboControllerSpawner class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_CONTROLLER_SPAWNER_H
#define TOROBO_CONTROLLER_SPAWNER_H


/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <string>
#include <map>
#include "torobo_control/torobo_action_controller_base.h"
#include "torobo_control/torobo_joint_state_controller.h"


namespace torobo {

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboControllerSpawner
{
public:
    ToroboControllerSpawner(ros::NodeHandle &nh);
    ~ToroboControllerSpawner();
    void run();

protected:
    struct ControllerParam
    {
        std::string type;
        double publish_rate;
        std::string joint;
        std::vector<std::string> joints;
    };
    std::map<std::string, ControllerParam> getControllerParam();

private:
    ros::NodeHandle& nh_;
    std::unique_ptr<ToroboJointStateController> torobo_joint_state_controller_;
    std::vector<std::unique_ptr<ToroboActionControllerBase> > controllers_;
};

} // namespace torobo

#endif /* TOROBO_CONTROLLER_SPAWNER_H */
