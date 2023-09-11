/**
 * @file  MasterControllerClient.h
 * @brief Master controller client class
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __MASTER_CONTROLLER_CLIENT_H__
#define __MASTER_CONTROLLER_CLIENT_H__

 /*----------------------------------------------------------------------
  Macro Definitions
  ----------------------------------------------------------------------*/
#define MCC_SEND_TIMEOUT_COUNT_TH  (100)
#define MCC_SEND_TIMEOUT_RETRY_NUM (5)

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include <stdint.h>
#include <string>
#include <vector>
#include <deque>
#include <unordered_map>
#include <mutex>
#include <memory>
#include "Communication/Packet/RecvPacket.h"
#include "Communication/Packet/SendPacket.h"
#include "Communication/Packet/UniqueSendPacketMap.h"
#include "Communication/Packet/send_packet_buffer.h"
#include "Communication/Packet/send_packet_merger.h"

  //!< Forward declaration
class AbstractClient;
class PacketParser;

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class MasterControllerClient
{
public:
    MasterControllerClient(int jointsNum);
    virtual ~MasterControllerClient();

    std::string GetVersion();

    bool InitAsTcpClient(const std::string ip, const int port);
    bool InitAsUdpClient(const std::string ip, const int port);
    bool InitAsSerialClient(const std::string deviceName, const uint32_t baudRate);

    void CreateIndependentSendPacketBuffer(const std::string& buffer_name);

    bool Connect();
    bool Disconnect() const;
    bool ReConnect(const int retryNum = 1);

    bool IsInit() const;
    bool IsConnected() const;
    int32_t GetRecvBufferSize() const;
    int32_t GetSendBufferSize() const;

    RecvPacket GetLastRecvPacket() const;

    virtual int32_t ReceiveStatus();
    virtual void SendPacketInBuffer();

    std::vector<std::string> GetCurStateJson();
    std::vector<RecvPacket> GetCurStatePacket();
    void PushSendQueue(SendPacket& packet, const bool confirmReceived = true);
    void PushFrontSendQueue(SendPacket& packet, const bool confirmReceived = true);
    bool PushSendQueueJson(const std::string& sendJson, const bool confirmReceived = true);

    void PushBackIndependentSendBuffer(SendPacket& packet, const std::string& independent_buffer_name, const bool confirmReceived = true, const bool is_unique = false, const std::string key="");

protected:
    void MergeIndependentSendPacketBuffer();
    void InsertUniqueSendMap(SendPacket& packet);
    virtual void PushRecvBuffer(RecvPacket packet);
    void FlushRecvBuffer();
    SendPacket PopSendQueue();
    void FlushSendBuffer();
    bool SendOnce(SendPacket& sendPacket, const bool reSend = false);

    AbstractClient* m_pClient;
    int m_jointsNum;
    bool m_isInit;
    bool m_isConnected;
    int32_t m_maxBuffSize = 4096;
    PacketParser* m_parser;
    std::vector<RecvPacket> m_recvPacketVec;
    std::deque<SendPacket> m_sendPacketQueue;
    std::map<std::string, std::shared_ptr<SendPacketBuffer>> independent_send_packet_buffers_;
    UniqueSendPacketMap m_uniqueSendPacketMap;

    SendPacket m_lastSendPacket;
    RecvPacket m_lastRecvPacket;
    uint64_t m_lastRecvHostTimeStamp;
    uint64_t m_sendingTimeStamp;
    bool m_confirmReceived;
    bool m_sendEnd;
    int32_t m_sendTimeCount;
    int32_t m_sendFailCount;

private:
    std::mutex* mtx_;
};

#endif
