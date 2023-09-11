/**
 * @file  yaml_parser.h
 * @brief YAML parser class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef YAML_PARSER_H
#define YAML_PARSER_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <string>
#include <map>
#include <vector>

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
namespace torobo_common
{

class YamlParser
{
public:
    typedef struct
    {
        std::string controllerName;
        std::string type;
        std::vector<std::string> joints;
    }ControllerConfig;

    YamlParser();
    virtual ~YamlParser();

    std::map<std::string, YamlParser::ControllerConfig> Parse(std::string yamlFileName);
    void DebugPrint(std::map<std::string, YamlParser::ControllerConfig> configMap);
};

}

#endif