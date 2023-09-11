/**
 * @file  RecvPacket.h
 * @brief Class of Recv Packet
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __RECV_PACKET_H__
#define __RECV_PACKET_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <stdint.h>
#include <vector>
#include "PacketDefine.h"

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
class RecvPacket
{
public:
    RecvPacket(int jointsNum);
    virtual ~RecvPacket();

    int GetDataSize();
    int GetPacketSize();
    bool Parse(std::vector<char> data);

    rxPreData_t m_preData;
    std::vector<rxJoint_t> m_joint;
private:
    int m_jointsNum;
    int m_preDataSize;
    int m_jointDataSize;
    int m_packetSize;
};

#endif
