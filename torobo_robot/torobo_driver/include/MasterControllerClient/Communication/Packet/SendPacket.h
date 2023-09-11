/**
 * @file  SendPacket.h
 * @brief Class of Send Packet
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __SEND_PACKET_H__
#define __SEND_PACKET_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <stdint.h>
#include <vector>
#include <string>
#include "PacketDefine.h"
#include "PacketOrder.h"

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class SendPacket
{
public:
    SendPacket(int jointsNum);
    virtual ~SendPacket();
    SendPacket(const SendPacket& obj);

    int GetDataSize() const;
    int GetPacketSize() const;
    std::vector<char> GetCharVector() const;
    bool GetConfirmReceived() const;
    void SetConfirmReceived(const bool isSet);
    bool IsUnique() const;
    void SetIsUnique(const bool isSet);
    const std::string& GetUniqueKey() const;
    void SetUniqueKey(const std::string& unique_key);

    uint64_t GetTimeStamp() const;
    void SetTimeStamp(const uint64_t timeStmap);
    void SetArmOrder(const ePacketArmOrder order);
    void SetArmOrder(const int order);
    void SetJointOrder(const int id, const ePacketOrder order);
    void SetJointOrder(const std::vector<int> idvec, const ePacketOrder order);
    void SetValue(const int id, const int valueNum, const float value);
    void SetValue(const std::vector<int> idvec, const int valueNum, const float value);
    void SetCommonCommand(const std::string& idstr, const ePacketOrder order, float value1 = 0.0f, float value2 = 0.0f, float value3 = 0.0f, float value4 = 0.0f);
    void SetCommonCommand(const std::string& idstr, const int order, float value1 = 0.0f, float value2 = 0.0f, float value3 = 0.0f, float value4 = 0.0f);

    void SetPayload(const float mass, const float massCenterX = 0.0f, const float massCenterY = 0.0f, const float massCenterZ = 0.0f);
    void SetPayloadInertiaTensorDiagonal(const float ixx, const float iyy, const float izz);
    void SetPayloadInertiaTensorTriangle(const float ixy, const float ixz, const float iyz);
    void SetGripperMaxEffort(const float maxEffortValue);

    void SetControlMode(const std::string& idstr, const int mode);
    void SetPositionControlMode(const std::string& idstr);
    void SetVelocityControlMode(const std::string& idstr);
    void SetCurrentControlMode(const std::string& idstr);
    void SetExternalForceFollowingControlMode(const std::string& idstr);
    void SetTrajectoryControlMode(const std::string& idstr);
    void SetOnlineTrajectoryControlMode(const std::string& idstr);
    void SetRotationFixedControlMode(const std::string& idstr);
    void SetServoOff(const std::string& idstr);
    void SetServoOn(const std::string& idstr);
    void SetBrakeOff(const std::string& idstr);
    void SetBrakeOn(const std::string& idstr);
    void SetReset(const std::string& idstr);
    void SetGetSlaveEffortOffset(const std::string& idstr);
    void SetParamKp(const std::string& idstr, const float value);
    void SetParamKi(const std::string& idstr, const float value);
    void SetParamKd(const std::string& idstr, const float value);
    void SetParamWindupLimit(const std::string& idstr, const float value);
    void SetParamVelocityOverride(const std::string& idstr, const float value);
    void SetParamVelocityMax(const std::string& idstr, const float value);
    void SetParamAccelerationMax(const std::string& idstr, const float value);
    void SetParamJerkMax(const std::string& idstr, const float value);
    void SetParamExtFFOnTrajSoftnessOverride(const std::string& idstr, const float value);
    void SetArbitraryParam(const std::string& idstr, const int paramNum, const float value);
    void SetRefCurrent(const std::string& idstr, const float refCur);
    void SetRefPosition(const std::string& idstr, const float refPos);
    void SetRefVelocity(const std::string& idstr, const float refVel);
    void SetRefPositionVelocity(const std::string& idstr, const float refPos, const float refVel);
    void SetRefPositionVelocityAcceleration(const std::string& idstr, const float refPos, const float refVel, const float refAcc);
    void SetDynamics(const std::string& idstr, const float gravityEffort, const float refDynamicsEffort=0.0f, const float curDynamicsEffort=0.0f, const float inertiaDiagElemnt=0.0f);
    void SetTrajectoryViaClear(const std::string& idstr);
    void SetTrajectoryControlStart(const std::string& idstr);
    void SetTrajectoryControlCancel(const std::string& idstr);
    void SetTrajectoryP_DoubleS(const std::string& idstr, const float refPos);
    void SetTrajectoryPT_LinSpline(const std::string& idstr, const float refPos, const float transitionTime, bool isAbsTime=true);
    void SetTrajectoryPT_Trapezoidal(const std::string& idstr, const float refPos, const float transitionTime, bool isAbsTime=true);
    void SetTrajectoryPT_DoubleS(const std::string& idstr, const float refPos, const float transitionTime, bool isAbsTime=true);
    void SetTrajectoryPVT(const std::string& idstr, const float refPos, const float refVel, const float transitionTime, bool isAbsTime = true);
    void SetTrajectoryPVAT(const std::string& idstr, const float refPos, const float refVel, const float refAcc, const float transitionTime, bool isAbsTime = true);
    void SetMoveHomePosition(const std::string& idstr);
    void SetGeneralRegisterType(const std::string& idstr, const int registerNumber, const int registerTypeNumber);

    txPreData_t m_preData;
    std::vector<txJoint_t> m_joint;

protected:
    bool IsValidID(const int id);
    bool IsValidValueNum(const int valueNum);
    std::vector<int> ParseIDString(const std::string& idstr);

private:
    int m_jointsNum;
    int m_preDataSize;
    int m_jointDataSize;
    int m_packetSize;
    bool m_confirmReceived;
    bool m_isUnique;
    std::string m_uniqueKey;
};

#endif
