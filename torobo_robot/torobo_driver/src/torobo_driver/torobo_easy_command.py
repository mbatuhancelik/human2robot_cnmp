#!/usr/bin/env python
import rospy
import sys
import numpy
from trajectory_msgs.msg import JointTrajectoryPoint
from torobo_driver import send_common_command
from torobo_driver import set_general_output_register_client

paramNameNumberDict_ = { \
    "kp": 1 ,
    "ki": 2 ,
    "kd": 3 ,
    "wl": 4 ,
    "dyna_coef": 10 ,
    "dyna_intercept": 11 ,
    "static_coef": 12 ,
    "damping_eft_th": 13 ,
    "torque_const": 20 ,
    "eft_th": 21 ,
    "gripper_max_effort": 50, "grippermaxeffort": 50 ,
    "velocity_override": 70, "velov": 70, "override": 70 ,
    "velocity_max": 71, "velmax": 71 ,
    "acceleration_max": 72, "accmax": 72 ,
    "jerk_max": 73, "jerkmax": 73 ,
    "gravity_torque": 200, "gravity_effort": 200 ,
    "dynamics_torque_cur": 201, "dynamics_effort_cur": 201 ,
    "dynamics_torque_ref": 202, "dynamics_effort_ref": 202 ,
    "ex_torque_cur": 203, "ex_effort_cur": 203 ,
    "ex_torque_ref": 204, "ex_effort_ref": 204 ,
    "out_conv_in_position": 300, "outConvInPosition": 300, "in_position": 300 ,
    "in_enc_position": 301,
    "in_velocity": 302,
    "in_acceleration": 303,
    "out_enc_position": 304,
    "power_current": 305,
    "power_consumption": 306,
    "ethertime": 500,
    "debugcounter0": 990,
    "debugcounter1": 991,
    "debugcounter2": 992,
    "debugcounter3": 993,
    "debugcounter4": 994,
}

def splitstring(string):
    sp = string.split("/")
    ar = []
    for s in sp:
        ar.append(s)
    return ar

class eWholeOrder:
    NONE = (0x00)
    SET_PAYLOAD = (0xA0)
    SET_PAYLOAD_INERTIA_DIAGONAL = (0xA1)
    SET_PAYLOAD_INERTIA_TRIANGLE = (0xA2)
    SET_GRIPPER_MAX_EFFORT = (0xB0)

class eJointOrder:
    NONE = (0x00)

    RESET = (0xA0)
    SERVO_OFF = (0xA1)
    SERVO_ON = (0xA2)
    CTRL_MODE = (0xA3)
    GET_SLAVE_EFFORT_OFFSET = (0xAB)

    PARAM_KP = (0xB0)
    PARAM_KI = (0xB1)
    PARAM_KD = (0xB2)
    PARAM_WINDUP_LIMIT = (0xB3)

    PACKET_ORDER_CTRL_PARAM_VELOCITY_OVERRIDE = (0xBA)
    PACKET_ORDER_CTRL_PARAM_VELOCITY_MAX      = (0xBB)
    PACKET_ORDER_CTRL_PARAM_ACCELERATION_MAX  = (0xBC)
    PACKET_ORDER_CTRL_PARAM_JERK_MAX          = (0xBD)

    ARBITRARY_PARAM= (0xBF)

    BRAKE_OFF = (0xC0)
    BRAKE_ON = (0xC1)

    MOVE_HOME_POSITION = (0xD0)

    CURRENT = (0x10)
    POSITION = (0x11)
    VELOCITY = (0x12)

    POS_VEL = (0x1A)
    POS_VEL_ACC = (0x1B)

    TRAJ_VIA_CLEAR = (0x30)
    TRAJ_CTRL_START = (0x35)
    TRAJ_CTRL_CANCEL = (0x36)

    TRAJ_VIA_ABS_TIME_APPEND_PT_LIN_SPLINE = (0x3A)
    TRAJ_VIA_ABS_TIME_APPEND_PT_LINEAR = (0x3B)
    TRAJ_VIA_ABS_TIME_APPEND_PT_DOUBLE_S = (0x3C)
    TRAJ_VIA_ABS_TIME_APPEND_PVT = (0x3D)
    TRAJ_VIA_ABS_TIME_APPEND_PVAT = (0x3E)

    TRAJ_VIA_REL_TIME_APPEND_PT_LIN_SPLINE = (0x4A)
    TRAJ_VIA_REL_TIME_APPEND_PT_LINEAR = (0x4B)
    TRAJ_VIA_REL_TIME_APPEND_PT_DOUBLE_S = (0x4C)
    TRAJ_VIA_REL_TIME_APPEND_PVT = (0x4D)
    TRAJ_VIA_REL_TIME_APPEND_PVAT = (0x4E)

    TRAJ_VIA_APPEND_P_DOUBLE_S = (0x50)

def ValidateCommand(cmd, length):
    isValid = False
    if(isinstance(length, list)):
        for l in length:
            if(len(cmd) == l):
                isValid = True
                break
    elif len(cmd) == length:
        isValid = True
    if(isValid == False):
        raise Exception("Invalid command length")

def ParseSpecialCommand(nameSpace, tag, cmd):
    if tag == "sleep":
        ValidateCommand(cmd, 1)
        sec = float(cmd[0])
        rospy.sleep(sec)
    else:
        return False
    return True

def ParseMasterCommand(tag, cmd):
    whole_order = 0
    joint_order = 0
    joint_name = ""
    value1 = 0.0
    value2 = 0.0
    value3 = 0.0
    value4 = 0.0

    if tag == "mass" or tag == "payload":
        ValidateCommand(cmd, [1, 4])
        whole_order = eWholeOrder.SET_PAYLOAD
        joint_name = "1"
        value1 = float(cmd[0])
        if len(cmd) == 4:
            value2 = float(cmd[1])
            value3 = float(cmd[2])
            value4 = float(cmd[3])
    elif tag == "inertiadiag":
        ValidateCommand(cmd, 3)
        whole_order = eWholeOrder.SET_PAYLOAD_INERTIA_DIAGONAL
        joint_name = "1"
        value1 = float(cmd[0])
        value2 = float(cmd[1])
        value3 = float(cmd[2])
    elif tag == "inertiatri":
        ValidateCommand(cmd, 3)
        whole_order = eWholeOrder.SET_PAYLOAD_INERTIA_TRIANGLE
        joint_name = "1"
        value1 = float(cmd[0])
        value2 = float(cmd[1])
        value3 = float(cmd[2])
    elif tag == "grippermaxeffort":
        ValidateCommand(cmd, 1)
        whole_order = eWholeOrder.SET_GRIPPER_MAX_EFFORT
        joint_name = "1"
        value1 = float(cmd[0])

    elif tag == "s":
        ValidateCommand(cmd, 2)
        joint_name = cmd[0]
        if (cmd[1] == "0") or (cmd[1] == "off"):
            joint_order = eJointOrder.SERVO_OFF
        elif (cmd[1] == "1") or (cmd[1] == "on"):
            joint_order = eJointOrder.SERVO_ON
    elif tag == "r":
        ValidateCommand(cmd, 2)
        joint_order = eJointOrder.CTRL_MODE
        joint_name = cmd[0]
        value1 = int(cmd[1])
    elif tag == "home":
        ValidateCommand(cmd, 2)
        joint_order = eJointOrder.MOVE_HOME_POSITION
        joint_name = cmd[0]
        value1 = float(cmd[1])
    elif tag == "reset":
        ValidateCommand(cmd, 1)
        joint_order = eJointOrder.RESET
        joint_name = cmd[0]
    elif tag == "brake":
        ValidateCommand(cmd, 2)
        joint_name = cmd[0]
        if (cmd[1] == "0") or (cmd[1] == "off"):
            joint_order = eJointOrder.BRAKE_OFF
        elif (cmd[1] == "1") or (cmd[1] == "on"):
            joint_order = eJointOrder.BRAKE_ON
    elif tag == "slveftoffset" or tag == "eftoffset":
        ValidateCommand(cmd, 1)
        joint_order = eJointOrder.GET_SLAVE_EFFORT_OFFSET
        joint_name = cmd[0]
    elif tag == "c":
        ValidateCommand(cmd, 2)
        joint_order = eJointOrder.CURRENT
        joint_name = cmd[0]
        value1 = float(cmd[1])
    elif tag == "v":
        ValidateCommand(cmd, 2)
        joint_order = eJointOrder.VELOCITY
        joint_name = cmd[0]
        value1 = float(cmd[1])
    elif tag == "pos":
        ValidateCommand(cmd, 2)
        joint_order = eJointOrder.POSITION
        joint_name = cmd[0]
        value1 = float(cmd[1])
    elif tag == "kp":
        ValidateCommand(cmd, 2)
        joint_order = eJointOrder.PARAM_KP
        joint_name = cmd[0]
        value1 = float(cmd[1])
    elif tag == "ki":
        ValidateCommand(cmd, 2)
        joint_order = eJointOrder.PARAM_KI
        joint_name = cmd[0]
        value1 = float(cmd[1])
    elif tag == "kd":
        ValidateCommand(cmd, 2)
        joint_order = eJointOrder.PARAM_KD
        joint_name = cmd[0]
        value1 = float(cmd[1])
    elif tag == "wl":
        ValidateCommand(cmd, 2)
        joint_order = eJointOrder.PARAM_WINDUP_LIMIT
        joint_name = cmd[0]
        value1 = float(cmd[1])
    elif tag == "velocity_override" or tag == "velov" or tag == "override":
        ValidateCommand(cmd, 2)
        joint_order = eJointOrder.PACKET_ORDER_CTRL_PARAM_VELOCITY_OVERRIDE
        joint_name = cmd[0]
        value1 = float(cmd[1])
    elif tag == "velocity_max" or tag == "velmax":
        ValidateCommand(cmd, 2)
        joint_order = eJointOrder.PACKET_ORDER_CTRL_PARAM_VELOCITY_MAX
        joint_name = cmd[0]
        value1 = float(cmd[1])
    elif tag == "acceleration_max" or tag == "accmax":
        ValidateCommand(cmd, 2)
        joint_order = eJointOrder.PACKET_ORDER_CTRL_PARAM_ACCELERATION_MAX
        joint_name = cmd[0]
        value1 = float(cmd[1])
    elif tag == "jerk_max" or tag == "jerkmax":
        ValidateCommand(cmd, 2)
        joint_order = eJointOrder.PACKET_ORDER_CTRL_PARAM_JERK_MAX
        joint_name = cmd[0]
        value1 = float(cmd[1])
    elif tag == "param":
        ValidateCommand(cmd, 3)
        joint_order = eJointOrder.ARBITRARY_PARAM
        joint_name = cmd[0]
        paramStr = cmd[1]
        if(paramStr.isdigit()):
            value1 = float(paramStr)
        else:
            if(paramNameNumberDict_.has_key(paramStr)):
                paramNumber = paramNameNumberDict_[paramStr]
                value1 = float(paramNumber)
            else:
                raise Exception("invalid parameter name")
        value2 = float(cmd[2])
    elif tag == "general":
        ValidateCommand(cmd, 3)
        joint_order = 224
        joint_name = cmd[0]
        value1 = float(cmd[1])
        paramStr = cmd[2]
        paramNumber = 0
        if(paramStr.isdigit()):
            paramNumber = float(paramStr)
        else:
            if(paramNameNumberDict_.has_key(paramStr)):
                paramNumber = paramNameNumberDict_[paramStr]
            else:
                raise Exception("invalid parameter name")
        value2 = float(paramNumber)
    elif tag == "tc":
        ValidateCommand(cmd, 1)
        joint_order = eJointOrder.TRAJ_VIA_CLEAR
        joint_name = cmd[0]
    elif tag == "ts":
        ValidateCommand(cmd, 1)
        joint_order = eJointOrder.TRAJ_CTRL_START
        joint_name = cmd[0]
    elif tag == "tx":
        ValidateCommand(cmd, 1)
        joint_order = eJointOrder.TRAJ_CTRL_CANCEL
        joint_name = cmd[0]
    elif tag == "tps":
        ValidateCommand(cmd, 2)
        joint_order = eJointOrder.TRAJ_VIA_APPEND_P_DOUBLE_S
        joint_name = cmd[0]
        value1 = float(cmd[1])
    elif tag == "tptl":
        ValidateCommand(cmd, 3)
        joint_order = eJointOrder.TRAJ_VIA_ABS_TIME_APPEND_PT_LINEAR
        joint_name = cmd[0]
        value1 = float(cmd[1])
        value2 = float(cmd[2])
    elif tag == "tpts":
        ValidateCommand(cmd, 3)
        joint_order = eJointOrder.TRAJ_VIA_ABS_TIME_APPEND_PT_DOUBLE_S
        joint_name = cmd[0]
        value1 = float(cmd[1])
        value2 = float(cmd[2])
    elif tag == "tptls":
        ValidateCommand(cmd, 3)
        joint_order = eJointOrder.TRAJ_VIA_ABS_TIME_APPEND_PT_LIN_SPLINE
        joint_name = cmd[0]
        value1 = float(cmd[1])
        value2 = float(cmd[2])
    elif tag == "tpvt":
        ValidateCommand(cmd, 4)
        joint_order = eJointOrder.TRAJ_VIA_ABS_TIME_APPEND_PVT
        joint_name = cmd[0]
        value1 = float(cmd[1])
        value2 = float(cmd[2])
        value3 = float(cmd[3])
    elif tag == "tpvat":
        ValidateCommand(cmd, 5)
        joint_order = eJointOrder.TRAJ_VIA_ABS_TIME_APPEND_PVAT
        joint_name = cmd[0]
        value1 = float(cmd[1])
        value2 = float(cmd[2])
        value3 = float(cmd[3])
        value4 = float(cmd[4])
    elif tag == "debug":
        joint_name = cmd[0]
        joint_order = int(cmd[1])
        if len(cmd) > 2:
            value1 = float(cmd[2])
        if len(cmd) > 3:
            value2 = float(cmd[3])
        if len(cmd) > 4:
            value3 = float(cmd[4])
        if len(cmd) > 5:
            value4 = float(cmd[5])
    else:
        raise Exception("Invalid tag")
    joint_name = splitstring(joint_name)
    return joint_name, whole_order, joint_order, value1, value2, value3, value4

def SendEasyCommand(ns, tag, cmd):
    try:
        print "-------------------------"
        print tag
        for c in cmd:
            print c
        print "-------------------------"

        if ParseSpecialCommand(ns, tag, cmd):
            return
        [joint_name, whole_order, joint_order, value1, value2, value3, value4] = ParseMasterCommand(tag, cmd)
        client = send_common_command.SendCommonCommandClient(ns)
        client.call_service(joint_name, whole_order, joint_order, value1, value2, value3, value4)
    except Exception, e:
        print e

def SendEasyCommandText(ns="", text=""):
    sp = text.split(" ")
    if len(sp) < 2:
        return
    tag = sp[0]
    cmd = []
    for i in range(len(sp)-1):
        cmd.append(sp[i+1])
    # print tag, cmd
    SendEasyCommand(ns, tag, cmd)

if __name__ == '__main__':
    cmdlen = len(sys.argv)

    ns = sys.argv[1]
    tag = sys.argv[2]

    cmd = []
    for i in range(cmdlen-3):
        cmd.append(sys.argv[i+3])

    SendEasyCommand(ns, tag, cmd)
