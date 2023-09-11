/**
 * @file  PacketOrder.h
 * @brief Definitions of Packet Order
 *
 * @par   Copyright © 2016 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __PACKET_ORDER_H__
#define __PACKET_ORDER_H__
/*----------------------------------------------------------------------
    Enum definitions
 ----------------------------------------------------------------------*/
typedef enum
{
    PACKET_ARM_ORDER_NONE                   = (0x00),
    PACKET_ARM_ORDER_SET_PAYLOAD            = (0xA0),
    PACKET_ARM_ORDER_SET_PAYLOAD_INERTIA_DIAGONAL = (0xA1),
    PACKET_ARM_ORDER_SET_PAYLOAD_INERTIA_TRIANGLE = (0xA2),
    PACKET_ARM_ORDER_SET_GRIPPER_MAX_EFFORT = (0xB0),
}ePacketArmOrder;

typedef enum
{
    PACKET_ORDER_NONE                       = (0x00),

    PACKET_ORDER_RESET                      = (0xA0),
    PACKET_ORDER_SERVO_OFF                  = (0xA1),
    PACKET_ORDER_SERVO_ON                   = (0xA2),
    PACKET_ORDER_CTRL_MODE                  = (0xA3),
    PACKET_ORDER_GET_SLAVE_EFFORT_OFFSET    = (0xAB),

    PACKET_ORDER_CTRL_PARAM_KP              = (0xB0),
    PACKET_ORDER_CTRL_PARAM_KI              = (0xB1),
    PACKET_ORDER_CTRL_PARAM_KD              = (0xB2),
    PACKET_ORDER_CTRL_PARAM_WINDUP_LIMIT    = (0xB3),

    PACKET_ORDER_CTRL_PARAM_VELOCITY_OVERRIDE = (0xBA),
    PACKET_ORDER_CTRL_PARAM_VELOCITY_MAX      = (0xBB),
    PACKET_ORDER_CTRL_PARAM_ACCELERATION_MAX  = (0xBC),
    PACKET_ORDER_CTRL_PARAM_JERK_MAX          = (0xBD),

    PACKET_ORDER_ARBITRARY_CTRL_PARAM       = (0xBF),

    PACKET_ORDER_BRAKE_OFF                  = (0xC0),
    PACKET_ORDER_BRAKE_ON                   = (0xC1),

    PACKET_ORDER_MOVE_HOME_POSITION         = (0xD0), 

    PACKET_ORDER_CURRENT                    = (0x10),
    PACKET_ORDER_POSITION                   = (0x11),
    PACKET_ORDER_VELOCITY                   = (0x12),

    PACKET_ORDER_POS_VEL                    = (0x1A),
    PACKET_ORDER_POS_VEL_ACC                = (0x1B),

    PACKET_ORDER_DYNAMICS                   = (0x20),

    PACKET_ORDER_TRAJ_VIA_CLEAR             = (0x30),
    PACKET_ORDER_TRAJ_CTRL_START            = (0x35),
    PACKET_ORDER_TRAJ_CTRL_CANCEL           = (0x36),

    PACKET_ORDER_TRAJ_VIA_ABS_TIME_APPEND_PT_LIN_SPLINE  = (0x3A),
    PACKET_ORDER_TRAJ_VIA_ABS_TIME_APPEND_PT_TRAPEZOIDAL = (0x3B),
    PACKET_ORDER_TRAJ_VIA_ABS_TIME_APPEND_PT_DOUBLE_S    = (0x3C),
    PACKET_ORDER_TRAJ_VIA_ABS_TIME_APPEND_PVT            = (0x3D),
    PACKET_ORDER_TRAJ_VIA_ABS_TIME_APPEND_PVAT           = (0x3E),

    PACKET_ORDER_TRAJ_VIA_REL_TIME_APPEND_PT_LIN_SPLINE  = (0x4A),
    PACKET_ORDER_TRAJ_VIA_REL_TIME_APPEND_PT_TRAPEZOIDAL = (0x4B),
    PACKET_ORDER_TRAJ_VIA_REL_TIME_APPEND_PT_DOUBLE_S    = (0x4C),
    PACKET_ORDER_TRAJ_VIA_REL_TIME_APPEND_PVT            = (0x4D),
    PACKET_ORDER_TRAJ_VIA_REL_TIME_APPEND_PVAT           = (0x4E),

    PACKET_ORDER_TRAJ_VIA_APPEND_P_DOUBLE_S              = (0x50),

    PACKET_ORDER_MPCOM_GENERAL_REGISTER_TYPE_SET    = (0xE0),

    PACKET_ORDER_SLAVE_CMD_PARAM_READ        = (0xF0),
    PACKET_ORDER_SLAVE_CMD_PARAM_WRITE       = (0xF1),
    PACKET_ORDER_SLAVE_CMD_PARAM_RESTORE_ALL = (0xFA),
    PACKET_ORDER_SLAVE_CMD_PARAM_RESTORE     = (0xFB),
    PACKET_ORDER_SLAVE_CMD_SET_SPECIAL_TASK  = (0xFC),
    PACKET_ORDER_SLAVE_CMD_PARAM_WRITE_TEMP  = (0xFE),
    PACKET_ORDER_SLAVE_CMD_PARAM_RESTORE_TEMP= (0xFF),
}ePacketOrder;


#endif