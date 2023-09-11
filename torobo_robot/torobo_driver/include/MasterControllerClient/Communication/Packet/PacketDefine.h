/**
 * @file  PacketDefine.h
 * @brief Definitions of Packet
 *
 * @par   Copyright © 2017 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __PACKET_DEFINE_H__
#define __PACKET_DEFINE_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <stdint.h>

/*----------------------------------------------------------------------
 Macro definitions
 ----------------------------------------------------------------------*/
#define MPCOM_HEADER1               (0xAB)  //!< Packet Header 1
#define MPCOM_HEADER2               (0xCD)  //!< Packet Header 2
#define MPCOM_CRC_SIZE              (2)
#define MPCOM_GENERAL_VALUE_NUM     (4)


/*----------------------------------------------------------------------
 Struct Definition
 ----------------------------------------------------------------------*/
#pragma pack(1)
typedef struct
{
    uint8_t               type;                 //!< Slave Mcu Type
    uint8_t               comStatus;            //!< 通信状態
    uint8_t               systemMode;           //!< システムモード
    uint8_t               ctrlMode;             //!< 制御モード
    uint32_t              ewStatus;             //!< エラーステータス
    uint8_t               trjStatus;            //!< 軌道制御ステータス
    uint16_t              trjViaRemain;         //!< 軌道制御経由点残数
    float                 refCurrent;           //!< 目標電流[A]
    float                 refPosition;          //!< 目標位置[deg] or [mm]
    float                 refVelocity;          //!< 目標速度[deg/s] or [mm/s]
    float                 refAcceleration;      //!< 目標加速度[deg/s^2] or [mm/s^2]
    float                 refEffort;            //!< 目標トルク[Nm]or力[N]
    float                 current;              //!< 現在電流 [A]
    float                 position;             //!< 現在角度 [deg]
    float                 velocity;             //!< 現在角速度 [deg/s]
    float                 outConvInVelocity;    //!< 入力軸速度から計算した出力軸速度[deg/s] or [mm/s]
    float                 acceleration;         //!< 出力軸現在加速度[deg/s^2] or [mm/s^2]
    float                 outConvInAcceleration;//!< 入力軸加速度から計算した出力軸加速度[deg/s^2] or [mm/s^2]
    float                 effort;               //!< 現在トルク[Nm]or力[N]
    float                 temperature;          //!< 現在温度 [℃]
    float                 general[MPCOM_GENERAL_VALUE_NUM];    //!< General purpose variables
} rxJoint_t;

typedef struct
{
    uint8_t ID;
    uint8_t jointOrder;
    float  value1;
    float  value2;
    float  value3;
    float  value4;
} txJoint_t;

typedef struct
{
    uint8_t   HEADER1;
    uint8_t   HEADER2;

    uint64_t  timeStamp;
    uint64_t  hostTimeStamp;
    float    duration;
} rxPreData_t;

typedef struct
{
    uint8_t   HEADER1;
    uint8_t   HEADER2;

    uint64_t  timeStamp;
    uint8_t   armOrder;
} txPreData_t;
#pragma pack()

#endif
