/**
 * @file  MasterControllerClientMock.h
 * @brief Master controller client mock class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __MASTER_CONTROLLER_CLIENT_MOCK_H__
#define __MASTER_CONTROLLER_CLIENT_MOCK_H__

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "MasterControllerClient/MasterControllerClient.h"
#include <vector>
#include <thread>
#include <memory>

  /*----------------------------------------------------------------------
   Class Definitions
   ----------------------------------------------------------------------*/
class TrajViaPoint
{
public:
    TrajViaPoint();
    TrajViaPoint(float pos, float vel=0.0f, float acc=0.0f, float t=0.0f);
    ~TrajViaPoint();
    float position;
    float velocity;
    float acceleration;
    float time;
};

class MasterControllerClientMock : public MasterControllerClient
{
public:
    MasterControllerClientMock(int jointsNum, int durationMilliSec=10);
    virtual ~MasterControllerClientMock();

    int32_t ReceiveStatus() override;
    void SendPacketInBuffer() override;
    void SetRecvBuffer(RecvPacket packet);
    void SetJointType(const int joint_id, const uint8_t joint_type);
    void SetJointType(const int joint_id, const std::string& joint_type);

protected:
    void ManageInnerStateThreadProc();
    void ManageTrajState();

    void ParseSendPacket(const SendPacket& packet);
    void ParseWholeOrder(const uint8_t wholeOrder,
        const float value1, const float value2, const float value3, const float value4);
    void ParseJointOrder(const uint8_t id,
        const uint8_t jointOrder, const float value1, const float value2, const float value3, const float value4);
    void TrajControlStart(const uint8_t id);

    std::unique_ptr<std::thread> m_thread;
    bool m_isThreadWorking;
    bool m_isThreadEnd;
    const int m_durationMilliSec;
    RecvPacket m_internalStatus;
    std::vector<std::deque<TrajViaPoint>> m_trajViaPoints;
    std::vector<TrajViaPoint> m_lastViaPoint;
    std::vector<TrajViaPoint> m_startPoint;

private:
    std::mutex mock_mtx_;
};


#endif
