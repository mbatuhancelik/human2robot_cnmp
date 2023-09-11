/**
 * @file  UniqueSendPacketMap.h
 * @brief Class of Unique Send Packet
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __UNIQUE_SEND_PACKET_MAP_H__
#define __UNIQUE_SEND_PACKET_MAP_H__

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "SendPacket.h"
#include <unordered_map>
#include <mutex>

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class UniqueSendPacketMap
{
public:
    UniqueSendPacketMap();
    ~UniqueSendPacketMap();

    void Insert(const std::string& key, const SendPacket& sendPacket);
    std::vector<std::string> GetNotSentKeys() const;
    SendPacket GetSendPacket(const std::string& key);

protected:
    std::unordered_map<std::string, SendPacket> m_sendPacketMap;
    std::unordered_map<std::string, bool> m_packetAlreadSent;
    std::mutex* mtx_;
};


#endif
