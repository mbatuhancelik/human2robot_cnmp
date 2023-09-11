#!/usr/bin/env python
## -*- coding: utf-8 -*-
import rospy
import numpy
import actionlib
from trajectory_msgs.msg import JointTrajectory
from torobo_msgs.srv import DeleteTeachingTrajectory
from torobo_msgs.srv import GetTeachingTrajectory
from torobo_msgs.srv import GetTeachingTrajectoryNames
from torobo_msgs.srv import RecordTeachingTrajectory
from torobo_msgs.msg import MoveTeachingTrajectoryAction, MoveTeachingTrajectoryGoal, MoveTeachingTrajectoryResult

def DeleteTeachingTrajectoryFromRosParam(nameSpace, trajName):
    try:
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        service = rospy.ServiceProxy(nameSpace + 'delete_teaching_trajectory', DeleteTeachingTrajectory)
        response = service(trajName)
        if response.success:
            rospy.loginfo('delete Traj[%s] succeeded' % (trajName))
        else:
            rospy.loginfo('delete Traj[%s] failed' % (trajName))
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s" % e)

def GetTeachingTrajectoryFromRosParam(nameSpace, trajName):
    try:
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        service = rospy.ServiceProxy(nameSpace + 'get_teaching_trajectory', GetTeachingTrajectory)
        response = service(trajName)
        if response.success:
            rospy.loginfo('get Traj[%s] succeeded' % (trajName))
            return response.trajectory
        else:
            rospy.loginfo('get Traj[%s] failed' % (trajName))
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s" % e)
    return None

def GetTeachingTrajectoryNamesFromRosParam(nameSpace):
    try:
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        service = rospy.ServiceProxy(nameSpace + 'get_teaching_trajectory_names', GetTeachingTrajectoryNames)
        response = service()
        if response.success:
            rospy.loginfo('get Traj names succeeded')
            print response.teachingTrajectoryNames
            return response.teachingTrajectoryNames
        else:
            rospy.loginfo('get Traj names failed')
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s" % e)
    return None

def RecordTeachingTrajectoryToRosParam(nameSpace, trajName, jointTrajectory):
    try:
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        service = rospy.ServiceProxy(nameSpace + 'record_teaching_trajectory', RecordTeachingTrajectory)
        response = service(trajName, jointTrajectory)
        rospy.loginfo('service call')
        if response.success:
            rospy.loginfo('record Traj[%s] succeeded' % (trajName))
        else:
            rospy.loginfo('record Traj[%s] failed' % (trajName))
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s" % e)

def connect_server(nameSpace, timeout=3):
    rospy.loginfo("connect move to teaching trajectory action server")
    if (nameSpace[-1] != "/"):
        nameSpace += "/"
    ac = actionlib.SimpleActionClient(
        nameSpace + 'move_teaching_trajectory',
        MoveTeachingTrajectoryAction
    )
    if (ac.wait_for_server(rospy.Duration(timeout)) == False):
        rospy.loginfo("failed to connect")
        return False
    rospy.loginfo("succeeded to connect")
    return True

def MoveToTeachingTrajectory(nameSpace, teachingTrajectoryName, timeout=3):
    rospy.loginfo("move to teaching trajectory action client")
    result = MoveTeachingTrajectoryResult()
    try:
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        ac = actionlib.SimpleActionClient(
            nameSpace + 'move_teaching_trajectory',
            MoveTeachingTrajectoryAction
        )
        if (ac.wait_for_server(rospy.Duration(timeout)) == False):
            rospy.loginfo("timeout")
            return result
        ac.wait_for_server()
        goal = MoveTeachingTrajectoryGoal()
        goal.teachingTrajectoryName = teachingTrajectoryName

        rospy.loginfo("[Move to teaching trajectory service] teaching trajectory name:%s" % (goal.teachingTrajectoryName))
        ac.send_goal(goal)
        # ac.wait_for_result()
        result = ac.get_result()
    except rospy.ServiceException, e:
        rospy.loginfo("Action call failed: %s" % e)
    return result

if __name__ == '__main__':
    pass