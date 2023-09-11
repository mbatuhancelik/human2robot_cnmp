#include <gtest/gtest.h>
#include <ros/package.h>
#include <torobo_common/yaml_parser.h>

using namespace std;

class YamlParserTestFixture : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
    }
    virtual void TearDown()
    {
    }
};

TEST_F(YamlParserTestFixture, ParseYaml)
{
    string packagePath = ros::package::getPath("torobo_common");
    string yamlPath = packagePath + "/test/resources/controllers.yaml";

    torobo_common::YamlParser parser;
    map<string, torobo_common::YamlParser::ControllerConfig> config = parser.Parse(yamlPath);
#if 0
    parser.DebugPrint(config);
#endif
    ASSERT_TRUE(config.count("torobo/right_arm_controller") > 0);
    ASSERT_TRUE(config.count("torobo/left_arm_controller") > 0);
    ASSERT_TRUE(config.count("torobo/torso_controller") > 0);
    ASSERT_TRUE(config.count("torobo/head_controller") > 0);
}
