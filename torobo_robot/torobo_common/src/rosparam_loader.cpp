/**
 * @file  rosparam_loader.cpp
 * @brief rosparam_loader class
 *
 * @par   Copyright © 2019 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "torobo_common/rosparam_loader.h"

using namespace std;

namespace torobo_common
{
/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
 bool getControllerJointsMap( std::map<std::string, std::vector<std::string>>& controller_joints_map, const ros::NodeHandle& nh)
 {
    vector<string> controller_list;
    nh.param<vector<string>>("controller_list", controller_list, vector<string>());

    if(controller_list.size() == 0)
    {
        ROS_WARN("rosparam [%s] is not found.", (nh.getNamespace() + "/controller_list").c_str());
        return false;
    }


    for(auto it = controller_list.begin(); it != controller_list.end(); ++it)
    {
        const string& controller_name = *it;

        // get joint name list
        vector<string> joints;
        string joint;
        nh.param<string>(controller_name + "/joint", joint, "");
        if(joint == "")
        {
            nh.param<string>(controller_name + "/joints", joint, "");
        }

        if(joint != "")
        {
            joints.push_back(joint);
        }
        else
        {
            nh.param<vector<string>>(controller_name + "/joints", joints, vector<string>());
        }
        if(joints.size() == 0)
        {
            continue;
        }

        // count joint size
        controller_joints_map[controller_name] = vector<string>();
        for(auto jit = joints.begin(); jit != joints.end(); ++jit)
        {
            controller_joints_map[controller_name].push_back(*jit);
        }
    }

    if(controller_joints_map.size() == 0)
    {
        return false;
    }

    return true;
}

}