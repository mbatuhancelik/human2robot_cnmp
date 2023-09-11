/**
 * @file  yaml_parser.cpp
 * @brief YAML parser class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include <iostream>
#include <torobo_common/yaml_parser.h>
#include <yaml-cpp/yaml.h>

using namespace std;

namespace torobo_common
{
/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
YamlParser::YamlParser()
{
}

YamlParser::~YamlParser()
{
}

std::map<std::string, YamlParser::ControllerConfig> YamlParser::Parse(std::string yamlFileName)
{
    YAML::Node root = YAML::LoadFile(yamlFileName);
    map<string, ControllerConfig> configMap;

    for(auto it = root.begin()->second.begin(); it != root.begin()->second.end(); ++it)
    {
        ControllerConfig config;

        if((*it)["name"])
        {
            std::string controller_name = (*it)["name"].as<std::string>();
            config.controllerName = controller_name;
        }
        if((*it)["type"])
        {
            std::string controller_type = (*it)["type"].as<std::string>();
            config.type = controller_type;
        }
        if((*it)["joints"])
        {
            std::vector<std::string> jointsVec = (*it)["joints"].as<std::vector<std::string>>();
            for(auto jitr = jointsVec.begin(); jitr != jointsVec.end(); ++jitr)
            {
                config.joints.push_back(*jitr);
            }
        }
        configMap.insert(make_pair(config.controllerName, config));
    }

    return configMap;
}

void YamlParser::DebugPrint(std::map<std::string, YamlParser::ControllerConfig> configMap)
{
    for(auto itr = configMap.begin(); itr != configMap.end(); ++itr)
    {
        std::cout << "controllerName: " << itr->second.controllerName << std::endl;
        std::cout << "type:   " << itr->second.type << std::endl;
        std::cout << "joints: " << std::endl;;
        for(auto jitr = itr->second.joints.begin(); jitr != itr->second.joints.end(); ++jitr)
        {
            std::cout << "  - " << *jitr << std::endl;
        }
    }
}

}