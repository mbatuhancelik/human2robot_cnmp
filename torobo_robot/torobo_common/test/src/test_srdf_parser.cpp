#include <gtest/gtest.h>
#include <ros/package.h>
#include <torobo_common/srdf_parser.h>

using namespace std;

class SrdfParserTestFixture : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
    }
    virtual void TearDown()
    {
    }
};

TEST_F(SrdfParserTestFixture, ParseToroboArmSrdf)
{
    std::string packagePath = ros::package::getPath("torobo_common");
    std::string srdfPath = packagePath + "/test/resources/toroboarm.srdf";

    torobo_common::SrdfParser::GroupStateMap state = torobo_common::SrdfParser::getGroupStateFromFile(srdfPath);

    ASSERT_TRUE(state.count("arm") > 0);
    ASSERT_TRUE(state["arm"].count("home_position") > 0);
    ASSERT_EQ("arm/joint_1", state["arm"]["home_position"].joint_names[0]);
    ASSERT_EQ("arm/joint_2", state["arm"]["home_position"].joint_names[1]);
    ASSERT_EQ("arm/joint_3", state["arm"]["home_position"].joint_names[2]);
    ASSERT_EQ("arm/joint_4", state["arm"]["home_position"].joint_names[3]);
    ASSERT_EQ("arm/joint_5", state["arm"]["home_position"].joint_names[4]);
    ASSERT_EQ("arm/joint_6", state["arm"]["home_position"].joint_names[5]);
    ASSERT_EQ("arm/joint_7", state["arm"]["home_position"].joint_names[6]);
    ASSERT_DOUBLE_EQ(0.0,    state["arm"]["home_position"].positions[0]);
    ASSERT_DOUBLE_EQ(0.0,    state["arm"]["home_position"].positions[1]);
    ASSERT_DOUBLE_EQ(0.0,    state["arm"]["home_position"].positions[2]);
    ASSERT_DOUBLE_EQ(0.0,    state["arm"]["home_position"].positions[3]);
    ASSERT_DOUBLE_EQ(0.0,    state["arm"]["home_position"].positions[4]);
    ASSERT_DOUBLE_EQ(0.0,    state["arm"]["home_position"].positions[5]);
    ASSERT_DOUBLE_EQ(0.0,    state["arm"]["home_position"].positions[6]);
}

TEST_F(SrdfParserTestFixture, ParseToroboSrdf)
{
    std::string packagePath = ros::package::getPath("torobo_common");
    std::string srdfPath = packagePath + "/test/resources/torobo.srdf";

    torobo_common::SrdfParser::GroupStateMap state = torobo_common::SrdfParser::getGroupStateFromFile(srdfPath);

    ASSERT_TRUE(state.count("right_arm") > 0);
    ASSERT_TRUE(state["right_arm"].count("home_position") > 0);
    ASSERT_EQ("right_arm/joint_1", state["right_arm"]["home_position"].joint_names[0]);
    ASSERT_EQ("right_arm/joint_2", state["right_arm"]["home_position"].joint_names[1]);
    ASSERT_EQ("right_arm/joint_3", state["right_arm"]["home_position"].joint_names[2]);
    ASSERT_EQ("right_arm/joint_4", state["right_arm"]["home_position"].joint_names[3]);
    ASSERT_EQ("right_arm/joint_5", state["right_arm"]["home_position"].joint_names[4]);
    ASSERT_EQ("right_arm/joint_6", state["right_arm"]["home_position"].joint_names[5]);
    ASSERT_DOUBLE_EQ(0.0,          state["right_arm"]["home_position"].positions[0]);
    ASSERT_DOUBLE_EQ(1.5708,       state["right_arm"]["home_position"].positions[1]);
    ASSERT_DOUBLE_EQ(0.0,          state["right_arm"]["home_position"].positions[2]);
    ASSERT_DOUBLE_EQ(0.0,          state["right_arm"]["home_position"].positions[3]);
    ASSERT_DOUBLE_EQ(0.0,          state["right_arm"]["home_position"].positions[4]);
    ASSERT_DOUBLE_EQ(0.0,          state["right_arm"]["home_position"].positions[5]);

    ASSERT_TRUE(state.count("left_arm") > 0);
    ASSERT_TRUE(state["left_arm"].count("home_position") > 0);
    ASSERT_EQ("left_arm/joint_1", state["left_arm"]["home_position"].joint_names[0]);
    ASSERT_EQ("left_arm/joint_2", state["left_arm"]["home_position"].joint_names[1]);
    ASSERT_EQ("left_arm/joint_3", state["left_arm"]["home_position"].joint_names[2]);
    ASSERT_EQ("left_arm/joint_4", state["left_arm"]["home_position"].joint_names[3]);
    ASSERT_EQ("left_arm/joint_5", state["left_arm"]["home_position"].joint_names[4]);
    ASSERT_EQ("left_arm/joint_6", state["left_arm"]["home_position"].joint_names[5]);
    ASSERT_DOUBLE_EQ(0.0,         state["left_arm"]["home_position"].positions[0]);
    ASSERT_DOUBLE_EQ(1.5708,      state["left_arm"]["home_position"].positions[1]);
    ASSERT_DOUBLE_EQ(0.0,         state["left_arm"]["home_position"].positions[2]);
    ASSERT_DOUBLE_EQ(0.0,         state["left_arm"]["home_position"].positions[3]);
    ASSERT_DOUBLE_EQ(0.0,         state["left_arm"]["home_position"].positions[4]);
    ASSERT_DOUBLE_EQ(0.0,         state["left_arm"]["home_position"].positions[5]);

    ASSERT_TRUE(state.count("torso") > 0);
    ASSERT_TRUE(state["torso"].count("home_position") > 0);
    ASSERT_EQ("torso/joint_1",      state["torso"]["home_position"].joint_names[0]);
    ASSERT_EQ("torso/joint_2",      state["torso"]["home_position"].joint_names[1]);
    ASSERT_DOUBLE_EQ(0.0,           state["torso"]["home_position"].positions[0]);
    ASSERT_DOUBLE_EQ(0.0,           state["torso"]["home_position"].positions[1]);

    ASSERT_TRUE(state.count("head") > 0);
    ASSERT_TRUE(state["head"].count("home_position") > 0);
    ASSERT_EQ("head/joint_1",      state["head"]["home_position"].joint_names[0]);
    ASSERT_EQ("head/joint_2",      state["head"]["home_position"].joint_names[1]);
    ASSERT_DOUBLE_EQ(0.0,           state["head"]["home_position"].positions[0]);
    ASSERT_DOUBLE_EQ(0.0,           state["head"]["home_position"].positions[1]);
}
