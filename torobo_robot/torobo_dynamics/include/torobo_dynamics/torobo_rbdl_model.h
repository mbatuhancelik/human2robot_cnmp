/**
 * @file  ToroboRbdlModel.h
 * @brief Torobo Rigid Body Dynamics Library class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef TOROBO_RBDL_MODEL_H
#define TOROBO_RBDL_MODEL_H

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <rbdl/rbdl.h>
#include <memory.h>
#include <vector>

namespace torobo
{

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class ToroboRbdlModel
{
public:
    ToroboRbdlModel();
    virtual ~ToroboRbdlModel();

    void UpdateQ    (const std::vector<std::string>& joint_names, std::vector<double> Q);
    void UpdateQDot (const std::vector<std::string>& joint_names, std::vector<double> QDot);
    void UpdateQDDot(const std::vector<std::string>& joint_names, std::vector<double> QDDot);
    void UpdateTau  (const std::vector<std::string>& joint_names, std::vector<double> Tau);

    void Print();
    int GetBodyId(const std::string& joint_name);
    int GetJointDoF();
    void CalcForwardDynamics();
    void CalcInverseDynamics();
    void CalcCompositeRigidBodyAlgorithm();

    std::vector<double> GetTau(const std::vector<std::string>& joint_names);
    std::vector<double> GetInertiaDiagonal(const std::vector<std::string>& joint_names);

    bool AddFixedBody(const std::string& prefix,int jointsNum,double mass,const RigidBodyDynamics::Math::Vector3d& com,const RigidBodyDynamics::Math::Matrix3d& InertiaMat);
    int ShowBody();

    RigidBodyDynamics::Math::VectorNd Q_;
    RigidBodyDynamics::Math::VectorNd QDot_;
    RigidBodyDynamics::Math::VectorNd QDDot_;
    RigidBodyDynamics::Math::VectorNd Tau_;
    RigidBodyDynamics::Math::MatrixNd H_;

protected:
    bool LoadModelFromUrdfFile(std::string urdfFilePath);
    bool LoadModelFromUrdfRosParam(std::string urdfParamName);
    void InitGravityVector();
    void InitBodyNameMap();
    bool GetLastLinkName(std::string& outLinkName, const std::string& prefix);

    std::unique_ptr<RigidBodyDynamics::Model> model_;
    std::map<std::string, int> bodyNameIdMap_;
    std::map<int, std::string> bodyIdNameMap_;
    std::map<std::string, int> addedBodyIndexMap_;
    std::map<std::string, RigidBodyDynamics::Body> parentBodyMap_;
private: };

}

#endif
