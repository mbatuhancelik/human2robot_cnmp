#include "torobo_driver/torobo_driver_nodecore.h"
#include "torobo_common/rosparam_loader.h"
#include "torobo_common/robot_description_parser.h"

using namespace std;

namespace torobo
{

//!< Forward declarations
static std::string CastXmlRpcValueAsString(XmlRpc::XmlRpcValue value);
static map<std::string, string> ParseXmlRpcStructToMap(XmlRpc::XmlRpcValue& value);

ToroboDriverNodeCore::ToroboDriverNodeCore(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh)
{
    // get params
    ToroboDriverCommonParam common_param;
    private_nh_.param<bool>("debug", debug_, false);
    private_nh_.param<bool>("mock", mock_, false);
    private_nh_.param<double>("rate", rate_, 100.0);
    private_nh_.param<double>("timeout_sec", timeout_sec_, 1.0);
    private_nh_.param<string>("robot_description_name", robot_description_name_, "");
    private_nh_.param<double>("allowed_start_tolerance", common_param.allowed_start_tolerance, 0.01);

    // initialize
    const string controller_config_param_name = "torobo_controller_config";
    const string robot_description_param_name = robot_description_name_;
    vector<pair<string, ToroboDriverParam>> param_vec;
    if(!GetControllerParamsFromRosParam(controller_config_param_name, robot_description_param_name, param_vec))
    {
        ROS_ERROR("[%s] failed to init.", ros::this_node::getName().c_str());
        return;
    }

    if(debug_)
    {
        PrintParam(param_vec);
    }

    // Create ToroboDrivers
    const chrono::nanoseconds period_ns(static_cast<int64_t>(1e+9 / rate_));
    for(auto itr = param_vec.begin(); itr!= param_vec.end(); ++itr)
    {
        unique_ptr<ToroboDriver> driver(new ToroboDriver(
            nh_, common_param, itr->second, period_ns, timeout_sec_, mock_, debug_)
        );
        drivers_.insert(make_pair(itr->first, std::move(driver)));
    }

    ROS_INFO("ToroboDriverNodeCore Ready.");
}

ToroboDriverNodeCore::~ToroboDriverNodeCore()
{
    // stop & join all ToroboDriver threads
    for(auto itr = drivers_.begin(); itr != drivers_.end(); ++itr)
    {
        itr->second->stop();
    }
    for(auto itr = threads_.begin(); itr != threads_.end(); ++itr)
    {
        (*itr)->join();
    }
}

void ToroboDriverNodeCore::run()
{
    for(auto itr = drivers_.begin(); itr != drivers_.end(); ++itr)
    {
        const unique_ptr<ToroboDriver>& driver = itr->second;
        if(!driver->Initialize())
        {
            ROS_ERROR("Fail to init torobo_driver");
            return;
        }

        threads_.emplace_back(new std::thread(std::bind(&ToroboDriver::run, driver.get())));
    }
}

bool ToroboDriverNodeCore::GetControllerParamsFromRosParam(const std::string& config_name, const std::string& robot_description_param_name, std::vector<std::pair<std::string, torobo::ToroboDriverParam>>& param_vec)
{
    if(!nh_.hasParam(config_name))
    {
        ROS_ERROR("rosparam name: [%s] is not found.", config_name.c_str());
        return false;
    }
    if(!nh_.hasParam(robot_description_param_name))
    {
        ROS_ERROR("rosparam name: [%s] is not found.", robot_description_param_name.c_str());
        return false;
    }

    // load torobo_controller_config 
    typedef XmlRpc::XmlRpcValue::ValueStruct::const_iterator XmlItr;
    XmlRpc::XmlRpcValue configs;
    nh_.getParam(config_name, configs);

    for(XmlItr it = configs.begin(); it != configs.end(); ++it)
    {
        const string controller_name = CastXmlRpcValueAsString(it->first);
        const string controller_config = config_name + "/" + controller_name;

        ToroboDriverParam param;
        if(!GetSignleControllerParamFromRosParam(controller_config, param))
        {
            return false;
        }
        param_vec.push_back(make_pair(controller_name, param));
    }

    // load controller joints map
    map<string, vector<string>> controller_joints_map;
    torobo_common::getControllerJointsMap(controller_joints_map, nh_);

    // load robot description
    torobo_common::RobotDescription robot_description;
    torobo_common::parseRobotDescriptionFromRosParam(robot_description, nh_);

    for(auto itr = param_vec.begin(); itr != param_vec.end(); ++itr)
    {
        ToroboDriverParam& param = itr->second;
        int joint_id_cnt = 0;
        for(auto it = param.controller.begin(); it != param.controller.end(); ++it)
        {
            const string& controller_name = it->first;
            if(controller_joints_map.count(controller_name) == 0)
            {
                ROS_ERROR("target controller [%s] is not generated.", controller_name.c_str());
                return false;
            }
            const vector<string>& joint_names = controller_joints_map.at(controller_name);
            const int joint_size = joint_names.size();
            if(joint_size == 0)
            {
                ROS_ERROR("target controller [%s] don't have any valid joint.", controller_name.c_str());
                return false;
            }

            it->second.joint_names = joint_names;
            for(int i = 0; i < joint_size; i++)
            {
                string joint_type = "revolute";
                for(int j = 0; j < robot_description.segments.size(); j++)
                {
                    if(joint_names[i] == robot_description.segments[j].joint_name_)
                    {
                        joint_type = robot_description.segments[j].joint_type_;
                        break;
                    }
                }
                it->second.joint_types.push_back(joint_type);
                it->second.joint_ids.push_back(joint_id_cnt);
                joint_id_cnt++;
            }
        }

        // Count all joints num
        param.allJointsNum = 0;
        for(auto itr = param.controller.begin(); itr != param.controller.end(); ++itr)
        {
            param.allJointsNum += (int)itr->second.joint_names.size();
        }
    }

    return true;
}

bool ToroboDriverNodeCore::GetSignleControllerParamFromRosParam(const std::string& controller_config_name, torobo::ToroboDriverParam& param)
{
    if(!nh_.hasParam(controller_config_name))
    {
        ROS_ERROR("rosparam name: [%s] is not found.", controller_config_name.c_str());
        return false;
    }
    XmlRpc::XmlRpcValue params;
    nh_.getParam(controller_config_name, params);
    
    for(int i = 0; i < params.size(); i++)
    {
        string controllerName = "";
        for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator itr = params[i].begin(); itr != params[i].end(); ++itr)
        {
            string name = itr->first;
            XmlRpc::XmlRpcValue value = itr->second;
            if(name == "connection_type")
            {
                param.interface = static_cast<string>(value);
            }
            else if(name == "ethernet")
            {
                map<string, string> m = ParseXmlRpcStructToMap(value);
                param.ip = m["master_controller_ip"];
                param.port = stoi(m["master_controller_port"]);
            }
            else if(name == "serial_port")
            {
                map<string, string> m = ParseXmlRpcStructToMap(value);
                param.com = m["master_controller_device_name"];
                param.baudrate = stoi(m["master_controller_baudrate"]);
            }
            else if(name == "controller_name")
            {
                controllerName = CastXmlRpcValueAsString(value);

                bool not_exist = true;
                for(auto itr = param.controller.begin(); itr != param.controller.end(); ++itr)
                {
                    if(itr->first == controllerName)
                    {
                        not_exist = false;
                        break;
                    }
                }
                if(not_exist)
                {
                    param.controller.push_back(make_pair(controllerName, ControllerParam()));
                }
            }
        }
    }

    return true;
}

void ToroboDriverNodeCore::PrintParam(const std::vector<std::pair<std::string,torobo::ToroboDriverParam>>& param_vec)
{
    for(auto pitr : param_vec)
    {
        const ToroboDriverParam& param = pitr.second;
        std::cout << "-----------------------------------------" << std::endl;
        std::cout << "JointsNum : " << param.allJointsNum << std::endl;
        std::cout << "Interface : " << param.interface <<  std::endl;
        std::cout << "IP        : " << param.ip <<  std::endl;
        std::cout << "Port      : " << param.port <<  std::endl;
        std::cout << "COM Port  : " << param.com <<  std::endl;
        std::cout << "Baudrate  : " << param.baudrate << std::endl;
        std::cout << "Controller:" <<  std::endl;
        for(auto itr = param.controller.begin(); itr != param.controller.end(); ++itr)
        {
            for(int i = 0; i < itr->second.joint_names.size(); i++)
            {
                std::cout << "    " << itr->second.joint_names[i] << ", " << itr->second.joint_ids[i]
                << ", Joint Type: " <<  itr->second.joint_types[i] << std::endl;
            }
        }
        std::cout << "-----------------------------------------" << std::endl;
    }
}

/*----------------------------------------------------------------------
  Static Method Definitions
 ----------------------------------------------------------------------*/
static std::string CastXmlRpcValueAsString(XmlRpc::XmlRpcValue value)
{
    switch(value.getType())
    {
    case XmlRpc::XmlRpcValue::TypeDouble:
        return to_string(static_cast<double>(value));
    case XmlRpc::XmlRpcValue::TypeInt:
        return to_string(static_cast<int>(value));
    case XmlRpc::XmlRpcValue::TypeBoolean:
        return to_string(static_cast<bool>(value));
    case XmlRpc::XmlRpcValue::TypeString:
        return static_cast<string>(value);
    };
    return "";
}

static map<std::string, string> ParseXmlRpcStructToMap(XmlRpc::XmlRpcValue& value)
{
    map<string, string> xmlMap;
    for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator itr = value.begin();
        itr != value.end(); ++itr)
    {
        string key = itr->first;
        XmlRpc::XmlRpcValue v = itr->second;
        xmlMap.insert(make_pair(key, CastXmlRpcValueAsString(v)));
    }

    return xmlMap;
}

}
