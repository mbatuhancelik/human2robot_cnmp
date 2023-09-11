#!/usr/bin/env python
## -*- coding: utf-8 -*-
import rospy
import numpy
import actionlib
from trajectory_msgs.msg import JointTrajectoryPoint
from torobo_msgs.msg import MoveHomePositionAction, MoveHomePositionGoal, MoveHomePositionResult
from srdfdom.srdf import SRDF


def ConvertPositionsToJointTrajectoryPoint(positions):
    point = JointTrajectoryPoint()
    for i in range(len(positions)):
        point.positions.append(positions[i])
        point.velocities.append(0.0)
        point.accelerations.append(0.0)
        point.effort.append(0.0)
    return point

def GetHomePosition(nameSpace):
    try:
        robot = SRDF.from_parameter_server()
        for group_state in robot.group_states:
            if group_state.name == 'home_position' and (group_state.group + '_controller') in nameSpace and len(group_state.joints) > 0:
                point = JointTrajectoryPoint()
                names = []
                for joint in group_state.joints:
                    point.positions.append(joint.value[0])
                    point.velocities.append(0.0)
                    point.accelerations.append(0.0)
                    point.effort.append(0.0)
                    names.append(joint.name)
                return point, names
        rospy.logwarn('home position is not defined for [%s]' % (nameSpace))
    except Exception as e:
        rospy.logerror("get home position failed: %s" % e)
    return None

def connect_server(nameSpace, timeout=3):
    rospy.loginfo("connect move home position action server")
    if (nameSpace[-1] != "/"):
        nameSpace += "/"
    ac = actionlib.SimpleActionClient(
        nameSpace + "move_home_position",
        MoveHomePositionAction
    )
    if (ac.wait_for_server(rospy.Duration(timeout)) == False):
        rospy.loginfo("failed to connect")
        return False
    rospy.loginfo("succeeded to connect")
    return True

def MoveToHomePosition(nameSpace, transitionTime, timeout=3):
    rospy.loginfo("move home position action client")
    result = MoveHomePositionResult()
    try:
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        ac = actionlib.SimpleActionClient(
            nameSpace + "move_home_position",
            MoveHomePositionAction
        )
        if (ac.wait_for_server(rospy.Duration(timeout)) == False):
            rospy.loginfo("timeout")
            return result
        goal = MoveHomePositionGoal()
        goal.transitionTime = transitionTime

        rospy.loginfo("[Move to home position service] transition time:%f" % goal.transitionTime)
        ac.send_goal(goal)
        # ac.wait_for_result()
        result = ac.get_result()
    except rospy.ServiceException, e:
        rospy.loginfo("Action call failed: %s" % e)
    return result


if __name__ == '__main__':
    pass