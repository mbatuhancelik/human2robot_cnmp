#include <ros/ros.h>
#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include "torobo_dynamics/torobo_rbdl_model.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace torobo;

void TestExpectedVectorValue(vector<double> actual, vector<double> expected)
{
    for(size_t size = 0; size < actual.size(); size++ )   
    {
        ASSERT_NEAR(actual[size],expected[size],FLT_EPSILON);
    }
}

class ToroboRbdlModelTestFixture : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        model_.reset(new ToroboRbdlModel());

        nameJointsNumMap.insert(make_pair(leftArmPrefix, 6));
        nameJointsNumMap.insert(make_pair(rightArmPrefix, 6));
        nameJointsNumMap.insert(make_pair(torsoHeadPrefix, 4));
    }
    virtual void TearDown()
    {
        nameJointsNumMap.clear();
    }

    std::unique_ptr<ToroboRbdlModel> model_;
    map<string, int> nameJointsNumMap;
    string leftArmPrefix = "left_arm";
    string rightArmPrefix = "right_arm";
    string torsoHeadPrefix = "torso_head";
    vector<string> rightArmJointNames  = { "right_arm/joint_1" ,"right_arm/joint_2","right_arm/joint_3", "right_arm/joint_4","right_arm/joint_5", "right_arm/joint_6"};
    vector<string> leftArmJointNames   = { "left_arm/joint_1"  ,"left_arm/joint_2" ,"left_arm/joint_3", "left_arm/joint_4","left_arm/joint_5", "left_arm/joint_6"};
    vector<string> torsoHeadJointNames = { "torso_head/joint_1","torso_head/joint_2","torso_head/joint_3", "torso_head/joint_4"};
};

TEST_F(ToroboRbdlModelTestFixture, TRBG_L_B_001_AddBodyTest)
{
    double mass = 0.76;
    Vector3d com(0.0, -0.007, 0.046);
    Matrix3d mat;
    mat << 1.104105e-3 , 2.054322e-5, -5.411983e-6,
            2.054322e-5, 7.765804e-4, 3.247528e-5,
            -5.411983e-6, 3.247528e-5,  6.346240e-4;
    
    ASSERT_EQ(16, model_->GetJointDoF()); 
    unsigned int leftFixedBodyId = model_->AddFixedBody(leftArmPrefix,6 , mass, com ,mat);
    unsigned int rightFixedBodyId = model_->AddFixedBody(rightArmPrefix,6 , mass, com ,mat);
    ASSERT_EQ(16, model_->GetJointDoF()); 

    // ASSERT_NEAR(mass,model_->model_->mFixedBodies[leftFixedBodyId].mMass, FLT_EPSILON);
    // ASSERT_TRUE(model_->model_->mFixedBodies[leftFixedBodyId].mCenterOfMass.isApprox(com));
    // ASSERT_TRUE(model_->model_->mFixedBodies[leftFixedBodyId].mInertia.isApprox(mat));

    // ASSERT_NEAR(mass,model_->model_->mFixedBodies[rightFixedBodyId].mMass, FLT_EPSILON);
    // ASSERT_TRUE(model_->model_->mFixedBodies[rightFixedBodyId].mCenterOfMass.isApprox(com));
    // ASSERT_TRUE(model_->model_->mFixedBodies[rightFixedBodyId].mInertia.isApprox(mat));

}

TEST_F(ToroboRbdlModelTestFixture, RemovePayloadTest)
{
    double mass = 0.76;
    Vector3d com(0.0, -0.007, 0.046);
    Matrix3d mat;
    mat << 1.104105e-3 , 2.054322e-5, -5.411983e-6,
            2.054322e-5, 7.765804e-4, 3.247528e-5,
            -5.411983e-6, 3.247528e-5,  6.346240e-4;
    
    model_->AddFixedBody(leftArmPrefix,6 , mass, com ,mat);
    model_->AddFixedBody(rightArmPrefix,6 , mass, com ,mat);
   
    double zmass = 0.0;
    Vector3d zcom = {0.0, 0.0, 0.0};
    Matrix3d zmat;
    zmat << 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0;

    unsigned int leftFixedBodyId = model_->AddFixedBody(leftArmPrefix,6 , zmass, zcom ,zmat);
    unsigned int rightFixedBodyId = model_->AddFixedBody(rightArmPrefix,6 ,zmass, zcom ,zmat);
    
    // ASSERT_NEAR(zmass, model_->model_->mFixedBodies[leftFixedBodyId].mMass, FLT_EPSILON);
    // ASSERT_TRUE(model_->model_->mFixedBodies[leftFixedBodyId].mCenterOfMass.isApprox(zcom));
    // ASSERT_TRUE(model_->model_->mFixedBodies[leftFixedBodyId].mInertia.isApprox(zmat));

    // ASSERT_NEAR(zmass, model_->model_->mFixedBodies[rightFixedBodyId].mMass, FLT_EPSILON);
    // ASSERT_TRUE(model_->model_->mFixedBodies[rightFixedBodyId].mCenterOfMass.isApprox(zcom));
    // ASSERT_TRUE(model_->model_->mFixedBodies[rightFixedBodyId].mInertia.isApprox(zmat));

}

TEST_F(ToroboRbdlModelTestFixture,CalcGravityTorque_Torso_0deg_ArmJ2_0deg) 
{
    vector<double> leftArmAngle   = { 0.0 , 0.0 , 0.0 , 0.0 , 0.0, 0.0};
    vector<double> rightArmAngle  = { 0.0 , 0.0 , 0.0 , 0.0 , 0.0, 0.0};
    vector<double> torsoHeadAngle = { 0.0 , 0.0 , 0.0 , 0.0 };
     
    model_->UpdateQ(leftArmJointNames  , leftArmAngle);
    model_->UpdateQ(rightArmJointNames , rightArmAngle);
    model_->UpdateQ(torsoHeadJointNames, torsoHeadAngle);

    model_->CalcInverseDynamics();

    TestExpectedVectorValue(
        vector<double>({ 0.0, -15.903444305, 0.0, -3.99915187,0.0 , -0.061781895}),
        model_->GetTau(leftArmJointNames)
    ); 
    TestExpectedVectorValue(
        vector<double>({ 0.0, -15.903444305, 0.0, -3.99915187,0.0 , -0.061781895}),
        model_->GetTau(rightArmJointNames)
    ); 
    TestExpectedVectorValue(
        vector<double>({ 0.0, 0.38432261 ,   0.0, -0.22947561 }),
        model_->GetTau(torsoHeadJointNames)
    ); 

}

TEST_F(ToroboRbdlModelTestFixture,CalcGravityTorque_Torso_0deg_ArmJ2_90deg) 
{
    vector<double> leftArmAngle   = { 0.0, M_PI / 2.0 , 0.0 , 0.0 , 0.0, 0.0};
    vector<double> rightArmAngle  = { 0.0, M_PI / 2.0 , 0.0 , 0.0 , 0.0, 0.0};
    vector<double> torsoHeadAngle = { 0.0, 0.0 , 0.0 , 0.0 };

    model_->UpdateQ(leftArmJointNames, leftArmAngle);
    model_->UpdateQ(rightArmJointNames, rightArmAngle);
    model_->UpdateQ(torsoHeadJointNames, torsoHeadAngle);

    model_->CalcInverseDynamics();

    TestExpectedVectorValue(
        vector<double>({ 0.0, 0.0, 0.0, 0.0 ,0.0 , 0.0}),
        model_->GetTau(leftArmJointNames)
    ); 
    TestExpectedVectorValue(
        vector<double>({ 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0}),
        model_->GetTau(rightArmJointNames)
    ); 
    TestExpectedVectorValue(
        vector<double>({ 0.0, 0.38432261 ,   0.0, -0.22947561 }),
        model_->GetTau(torsoHeadJointNames)
    );
} 


TEST_F(ToroboRbdlModelTestFixture,CalcGravityTorque_Torso_90deg_ArmJ1J2_90deg) 
{
    vector<double> leftArmAngle   = { M_PI/2.0 , M_PI / 2.0 , 0.0 , 0.0, 0.0, 0.0};
    vector<double> rightArmAngle  = { M_PI/2.0 , M_PI / 2.0 , 0.0 , 0.0, 0.0, 0.0};
    vector<double> torsoHeadAngle = { 0.0, 0.0 , 0.0 , 0.0 };
    
    model_->UpdateQ(leftArmJointNames, leftArmAngle);
    model_->UpdateQ(rightArmJointNames, rightArmAngle);
    model_->UpdateQ(torsoHeadJointNames, torsoHeadAngle);

    model_->CalcInverseDynamics();

    TestExpectedVectorValue(
        vector<double>({ 17.374441805 , 0.0, 0.0, 0.0 ,0.0 , 0.0}),
        model_->GetTau(leftArmJointNames)
    ); 
    TestExpectedVectorValue(
        vector<double>({ 17.374441805 , 0.0, 0.0, 0.0, 0.0 , 0.0}),
        model_->GetTau(rightArmJointNames)
    ); 
    TestExpectedVectorValue(
        vector<double>({ 0.0,  -34.364561 ,  0.0, -0.22947561 }),
        model_->GetTau(torsoHeadJointNames)
    );
}


TEST_F(ToroboRbdlModelTestFixture, CalcGravityTorque_Torso_0deg_ArmJ2_0deg_with_TRBG_L_B_001)
{
    vector<double> leftArmAngle   = { 0.0 , 0.0 , 0.0 , 0.0 , 0.0, 0.0};
    vector<double> rightArmAngle  = { 0.0 , 0.0 , 0.0 , 0.0 , 0.0, 0.0};
    vector<double> torsoHeadAngle = { 0.0 , 0.0 , 0.0 , 0.0 };
     
    model_->UpdateQ(leftArmJointNames  , leftArmAngle);
    model_->UpdateQ(rightArmJointNames , rightArmAngle);
    model_->UpdateQ(torsoHeadJointNames, torsoHeadAngle);

    Vector3d com(0.0, 0.0, 0.046);
    Matrix3d mat;
    mat << 1.104105e-3 , 2.054322e-5, -5.411983e-6,
            2.054322e-5, 7.765804e-4, 3.247528e-5,
            -5.411983e-6, 3.247528e-5,  6.346240e-4;

    model_->AddFixedBody(leftArmPrefix,nameJointsNumMap[leftArmPrefix] ,0.76, com ,mat);
    model_->AddFixedBody(rightArmPrefix,nameJointsNumMap[rightArmPrefix] , 0.76, com ,mat);

    model_->CalcInverseDynamics();
    
    TestExpectedVectorValue(
        vector<double>({ 0.0, -20.3603705970008413, 0.0, -6.5555493920004553, 0.0, -0.829446457}),
        model_->GetTau(leftArmJointNames)
    ); 
    TestExpectedVectorValue(
        vector<double>({ 0.0, -20.3603705970008413, 0.0, -6.5555493920004553, 0.0, -0.829446457}),
        model_->GetTau(rightArmJointNames)
    ); 
    TestExpectedVectorValue(
        vector<double>({ 0.0, 0.38432261 , 0.0, -0.22947561}),
        model_->GetTau(torsoHeadJointNames)
    );   
}

/*------------------------------------------------------------------------------
This test case does not ensure calculation accuracy of inertia matrix.
このテストケースは質量行列の計算があっているかどうか確かめるためのものではありません。
Expectedに使用した値はRBDLのCompositeRigidBodyAlgorithmの計算結果です。
-------------------------------------------------------------------------------*/
TEST_F(ToroboRbdlModelTestFixture, CalcInertiaDiagonalElement_Torso_0deg_ArmJ2_0deg)
{
    model_->CalcCompositeRigidBodyAlgorithm();

    TestExpectedVectorValue(
        vector<double>({0.01300034349998433966 ,0.57870038790001077089 ,0.00511252649999988009 ,0.08971013190000316773 ,0.00185596549999977010 ,0.00456334699999999972 }),
        model_->GetInertiaDiagonal(leftArmJointNames)
    ); 
    TestExpectedVectorValue(
        vector<double>({0.01300034350000755026 ,0.57870038790001077089 ,0.00511252649999988009 ,0.08971013190000316773 ,0.00185596549999977010 ,0.00456334699999999972 }),
        model_->GetInertiaDiagonal(rightArmJointNames)
    ); 
    TestExpectedVectorValue(
        vector<double>({3.25110229149972695950 ,1.93161795299928296110 ,0.00359752410000062622 ,0.00452963299999999992 }),
        model_->GetInertiaDiagonal(torsoHeadJointNames)
    );   
 
    
}


