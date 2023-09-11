#!/usr/bin/env python
## -*- coding: utf-8 -*-
import rospy
import numpy
import actionlib
import os
import rosparam
import yaml
from trajectory_msgs.msg import JointTrajectoryPoint
from torobo_msgs.srv import DeleteTeachingPoint
from torobo_msgs.srv import GetTeachingPoint
from torobo_msgs.srv import GetTeachingPointNames
from torobo_msgs.srv import RecordTeachingPoint
from torobo_msgs.msg import MoveTeachingPointAction, MoveTeachingPointGoal, MoveTeachingPointResult

def ConvertPositionsToJointTrajectoryPoint(positions):
    point = JointTrajectoryPoint()
    for i in range(len(positions)):
        point.positions.append(positions[i])
        point.velocities.append(0.0)
        point.accelerations.append(0.0)
        point.effort.append(0.0)
    return point

def DeleteTeachingPointFromRosParam(nameSpace, tpName):
    try:
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        service = rospy.ServiceProxy(nameSpace + 'delete_teaching_point', DeleteTeachingPoint)
        response = service(tpName)
        if response.success:
            rospy.loginfo('delete TP[%s] succeeded' % (tpName))
        else:
            rospy.loginfo('delete TP[%s] failed' % (tpName))
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s" % e)

def GetTeachingPointFromRosParam(nameSpace, tpName):
    try:
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        service = rospy.ServiceProxy(nameSpace + 'get_teaching_point', GetTeachingPoint)
        response = service(tpName)
        if response.success:
            rospy.loginfo('get TP[%s] succeeded' % (tpName))
            return response.point
        else:
            rospy.loginfo('get TP[%s] failed' % (tpName))
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s" % e)
        return
    return None

def GetTeachingPointNamesFromRosParam(nameSpace):
    try:
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        service = rospy.ServiceProxy(nameSpace + 'get_teaching_point_names', GetTeachingPointNames)
        response = service()
        if response.success:
            rospy.loginfo('get TP names succeeded')
            print response.teachingPointNames
            return response.teachingPointNames
        else:
            # Any teaching point has not created yet.
            #rospy.loginfo('get TP names failed')
            return []
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s" % e)
        return
    return None

def RecordTeachingPointToRosParam(nameSpace, tpName, point):
    try:
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        service = rospy.ServiceProxy(nameSpace + 'record_teaching_point', RecordTeachingPoint)
        response = service(tpName, point)
        if response.success:
            rospy.loginfo('record TP[%s] succeeded' % (tpName))
        else:
            rospy.loginfo('record TP[%s] failed' % (tpName))
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s" % e)

def connect_server(nameSpace, timeout=3):
    rospy.loginfo("connect move to teaching point action server")
    if (nameSpace[-1] != "/"):
        nameSpace += "/"
    ac = actionlib.SimpleActionClient(
        nameSpace + 'move_teaching_point',
        MoveTeachingPointAction
    )
    if (ac.wait_for_server(rospy.Duration(timeout)) == False):
        rospy.loginfo("failed to connect")
        return False
    rospy.loginfo("succeeded to connect")
    return True

def MoveToTeachingPoint(nameSpace, teachingPointName, transitionTime, timeout=3):
    rospy.loginfo("move to teaching point action client")
    result = MoveTeachingPointResult()
    try:
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        ac = actionlib.SimpleActionClient(
            nameSpace + 'move_teaching_point',
            MoveTeachingPointAction
        )
        if (ac.wait_for_server(rospy.Duration(timeout)) == False):
            rospy.loginfo("timeout")
            return result
        goal = MoveTeachingPointGoal()
        goal.teachingPointName = teachingPointName
        goal.transitionTime = transitionTime

        rospy.loginfo("[Move to teaching point service] teaching point name:%s, transition time:%f" % (goal.teachingPointName, goal.transitionTime))
        ac.send_goal(goal)
        # ac.wait_for_result()
        result = ac.get_result()
    except rospy.ServiceException, e:
        rospy.loginfo("Action call failed: %s" % e)
    return result

def LoadTeachinPointsToRosParamFromYaml(file_path):
    if(os.path.exists(file_path) == False):
        rospy.logerr("[%s] is not exists." % file_path)
        return False
    loaded = False
    paramlist = rosparam.load_file(file_path)
    for params, ns in paramlist:
        for k, v in params.items():
            if not 'controller' in k:
                continue
            if 'teaching_points' in v:
                rosparam.upload_params(ns + k + '/teaching_points', v['teaching_points'])
                loaded = True
    if not loaded:
        rospy.logerr("Teaching points not found in [%s]" % file_path)
    return True

def DampTeachingPointsToYamlFromRosParam(controller_name_list, file_path, overwrite=False):
    if file_path == "":
        rospy.logerr('Given file path is empty.')
        return False
    if((os.path.exists(file_path)) and (not overwrite)):
        rospy.logerr('Given file path already exists and overwrite flag is False.')
        return False
    save_param_exists = False
    param = {}
    for controller_name in controller_name_list:
        param[controller_name] = {}
        tps = rospy.get_param(controller_name + "/teaching_points", None)
        if (tps != None):
            param[controller_name]['teaching_points'] = tps
            save_param_exists = True

    if not save_param_exists:
        rospy.logerr('Any teaching points is not exists in rosparam.')
        return False
    with open(file_path, "w") as f:
        yaml.dump(param, f, default_flow_style=False)
    return True

if __name__ == '__main__':
    rospy.init_node('test_node')
    LoadTeachinPointsToRosParamFromYaml('/home/sato/catkin_ws/teaching_points/torobo_teaching_param.yaml')
    DampTeachingPointsToYamlFromRosParam(['left_arm_controller'], '/home/sato/catkin_ws/teaching_points/test.yaml', overwrite=True)
