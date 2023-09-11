/**
 * @file  send_packet_buffer.h
 * @brief Class of Send Packet Buffer
 *
 * @par   Copyright © 2019 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef SEND_PACKET_BUFFER_H
#define SEND_PACKET_BUFFER_H

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "MasterControllerClient/Communication/Packet/SendPacket.h"
#include <vector>
#include <map>
#include <unordered_map>
#include <mutex>

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class SendPacketBuffer
{
public:
    SendPacketBuffer();
    ~SendPacketBuffer();

    void pushBack(const SendPacket& packet);
    void popAllPackets(std::unordered_map<std::string, SendPacket>& unique_packets, std::vector<SendPacket>& not_unique_packets);
    int getBufferSize() { return buf_.size(); }
    int getUniqueKeyMapSize() { return unique_key_idx_map_.size(); }

protected:
    void reset();

    int idx_;
    std::map<int, SendPacket> buf_;
    std::unordered_map<std::string, int> unique_key_idx_map_;

    std::mutex mtx_;
};

#endif
