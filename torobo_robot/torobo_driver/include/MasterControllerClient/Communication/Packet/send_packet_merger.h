/**
 * @file  send_packet_merger.h
 * @brief Class of Send Packet Merger
 *
 * @par   Copyright © 2019 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef SEND_PACKET_MERGER_H
#define SEND_PACKET_MERGER_H

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "send_packet_buffer.h"
#include <string>
#include <deque>
#include <vector>
#include <memory>
#include <unordered_map>
#include <mutex>

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class SendPacketMerger
{
private:
    SendPacketMerger(){}
    ~SendPacketMerger(){}

public:
    typedef std::unordered_map<std::string, SendPacket> UniquePackets;
    typedef std::vector<SendPacket> NotUniquePackets;

    static std::deque<SendPacket> getMergedPacket(std::vector<std::shared_ptr<SendPacketBuffer>>& buffers, const int all_joints_size);
    static std::deque<SendPacket> mergeUniquePacket(std::vector<UniquePackets>& unique_packets_vec);
    static std::deque<SendPacket> mergeNotUniquePacket(std::vector<NotUniquePackets>& packets_vec, const int all_joints_size);
    static bool mergePacket(SendPacket& dst_packet, const SendPacket& src_packet);
};

#endif
