#!/usr/bin/env python
## -*- coding: utf-8 -*-
import os
import rospy
import rosparam
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *

import numpy
import yaml

import torobo_driver.torobo_easy_command
from torobo_msgs.msg import ToroboJointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from torobo_driver import servo_on_client
from torobo_driver import servo_off_client
from torobo_driver import error_reset_client
from torobo_driver import set_control_mode_client
from torobo_motion_manager import home_position_manager
from torobo_motion_manager import teaching_point_manager
from torobo_motion_manager import teaching_trajectory_manager

import torobo_common.rosparam_loader as rosparam_loader

class ToroboWholeBodyManager(Plugin):
    def __init__(self, context):
        super(self.__class__, self).__init__(context)
        self.setObjectName('ToroboWholeBodyManager')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()

        # load ui
        ui_file = os.path.join(rospkg.RosPack().get_path('torobo_gui'), 'resource', 'torobo_whole_body_manager.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ToroboWholeBodyManagerUi')

        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self.serial_number = context.serial_number()
        ns = rospy.get_namespace()
        self.create(ns, 200)

    def shutdown_plugin(self):
        self.destroy()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
    
    def create(self, nameSpace, updateTimeInterval):
        self.nameSpace = nameSpace
        self.controller_joint_names_dict = rosparam_loader.get_controller_name_joint_names_dict("controller_list")

        self.toroboJointStateJointNames = {}
        self.goHomeActionServerConnection = False
        self.moveTeachingPointActionServerConnection = False
        self.moveTeachingTrajectoryActionServerConnection = False

        self.CreateSubscriber(self.controller_joint_names_dict.keys())
        self.CreateServiceClient(self.controller_joint_names_dict.keys())

        self.initMainForm()

        self.updateTimer = QTimer(self)
        self.updateTimer.timeout.connect(self.update)
        self.updateTimer.start(updateTimeInterval)

        self.recordTimer = QTimer(self)
        self.recordTimer.timeout.connect(self.TrajectoryRecordCallback)
        self.recordPointsNum = 0

    def destroy(self):
        self.updateTimer.stop()
        self.recordTimer.stop()

    def CreateSubscriber(self, controller_list):
        self.toroboJointState = {}
        self.toroboJointStateSub = {}
        for k in controller_list:
            self.toroboJointStateSub[k] = rospy.Subscriber("joint_state_server/" + k + "/torobo_joint_state", ToroboJointState, self.toroboJointStateSubCallback, callback_args=k, queue_size=1)

    def CreateServiceClient(self, controller_list):
        self.servoOffClient = {}
        self.servoOnClient = {}
        self.errorResetClient = {}
        self.setControlModeClient = {}

        for k in controller_list:
            self.servoOffClient[k] = servo_off_client.ServoOffClient(k)
            self.servoOnClient[k] = servo_on_client.ServoOnClient(k)
            self.errorResetClient[k] = error_reset_client.ErrorResetClient(k)
            self.setControlModeClient[k] = set_control_mode_client.SetControlModeClient(k)
    
    def initMainForm(self):
        # button
        self._widget.buttonAllServoOn.clicked.connect(self.button_clicked)
        self._widget.buttonAllServoOff.clicked.connect(self.button_clicked)
        self._widget.buttonAllErrorReset.clicked.connect(self.button_clicked)
        self._widget.buttonTeachingMode.clicked.connect(self.button_clicked)
        self._widget.buttonMovingMode.clicked.connect(self.button_clicked)
        self._widget.buttonGoHome.clicked.connect(self.button_clicked)
        self._widget.buttonGetTp.clicked.connect(self.button_clicked)
        self._widget.buttonMoveTp.clicked.connect(self.button_clicked)
        self._widget.buttonTrajRecord.clicked.connect(self.button_clicked)
        self._widget.buttonTrajRun.clicked.connect(self.button_clicked)
        self._widget.buttonSaveRosParam.clicked.connect(self.button_clicked)
        self._widget.buttonLoadRosParam.clicked.connect(self.button_clicked)

    def toroboJointStateSubCallback(self, toroboJointState, name):
        self.toroboJointState[name] = toroboJointState

    def update(self):
        for k in self.controller_joint_names_dict.keys():
            v = self.toroboJointState[k]
            if v is None:
                continue
            errorJoint = []
            servoState = "Stop"
            servoStateColor = QColor(Qt.black)
            mode = "Unknown"
            modeColor = QColor(Qt.black)
            for i in range(len(v.name)):
                systemMode = v.systemMode[i]
                if systemMode == eSystemMode.ERROR:
                    servoState = "Error"
                    servoStateColor = QColor(Qt.red)
                    errorJoint.append(i+1)
                elif errorJoint == [] and systemMode == eSystemMode.RUN:
                    servoState = "Run"
                    servoStateColor = QColor(Qt.blue)
            if servoState == "Error":
                for i,j in enumerate(errorJoint):
                    if i == 0:
                        servoState += " "
                    else:
                        servoState += "/"
                    servoState += str(j)

            controlMode = v.ctrlMode[0]
            if controlMode == eHighCtrlMode.TRAJ:
                mode = "Moving"
                modeColor = QColor(Qt.red)
            elif controlMode == eHighCtrlMode.EXTERNAL_FORCE_FOLLOWING:
                mode = "Teaching"
                modeColor = QColor(Qt.blue)

            if "left_arm" in k:
                self._widget.labelLArmMode.setText(mode)
                self._widget.labelLArmServoState.setText(servoState)
                pal = self._widget.labelLArmServoState.palette()
                pal.setColor(QPalette.Foreground,servoStateColor)
                self._widget.labelLArmServoState.setPalette(pal)
                pal = self._widget.labelLArmMode.palette()
                pal.setColor(QPalette.Foreground,modeColor)
                self._widget.labelLArmMode.setPalette(pal)
            elif "right_arm" in k:
                self._widget.labelRArmMode.setText(mode)
                self._widget.labelRArmServoState.setText(servoState)
                pal = self._widget.labelRArmServoState.palette()
                pal.setColor(QPalette.Foreground,servoStateColor)
                self._widget.labelRArmServoState.setPalette(pal)
                pal = self._widget.labelRArmMode.palette()
                pal.setColor(QPalette.Foreground,modeColor)
                self._widget.labelRArmMode.setPalette(pal)
            elif "torso" in k:
                self._widget.labelTorsoMode.setText(mode)
                self._widget.labelTorsoServoState.setText(servoState)
                pal = self._widget.labelTorsoServoState.palette()
                pal.setColor(QPalette.Foreground,servoStateColor)
                self._widget.labelTorsoServoState.setPalette(pal)
                pal = self._widget.labelTorsoMode.palette()
                pal.setColor(QPalette.Foreground,modeColor)
                self._widget.labelTorsoMode.setPalette(pal)
            elif "head" in k:
                self._widget.labelHeadMode.setText(mode)
                self._widget.labelHeadServoState.setText(servoState)
                pal = self._widget.labelHeadServoState.palette()
                pal.setColor(QPalette.Foreground,servoStateColor)
                self._widget.labelHeadServoState.setPalette(pal)
                pal = self._widget.labelHeadMode.palette()
                pal.setColor(QPalette.Foreground,modeColor)
                self._widget.labelHeadMode.setPalette(pal)

    def button_clicked(self):
        sender = self.sender()
        tpNum = int(self._widget.spinBoxTpNumber.text())
        tpName = "tp" + str(tpNum)
        trajName = unicode(self._widget.lineEditTrajName.text())
        transitionTime = float(self._widget.doubleSpinBoxTransitionTime.text())
        recordInterval = float(self._widget.doubleSpinBoxRecordInterval.text())

        def CallClassServiceAllController(classDict, sleepTime=0.0):
            for (name, c) in classDict.items():
                c.call_service("all")
                rospy.sleep(sleepTime)

        def CallClassServiceWithArgAllController(classDict, arg, sleepTime=0.0):
            for (name, c) in classDict.items():
                c.call_service("all", arg)
                rospy.sleep(sleepTime)

        if sender == self._widget.buttonAllServoOn:
            CallClassServiceAllController(self.servoOnClient, sleepTime=0.2)
        elif sender == self._widget.buttonAllServoOff:
            CallClassServiceAllController(self.servoOffClient)
        elif sender == self._widget.buttonAllErrorReset:
            CallClassServiceAllController(self.errorResetClient)
        elif sender == self._widget.buttonTeachingMode:
            CallClassServiceWithArgAllController(self.setControlModeClient, "external_force_following")
        elif sender == self._widget.buttonMovingMode:
            CallClassServiceWithArgAllController(self.setControlModeClient, "position")
        elif sender == self._widget.buttonGetTp:
            for name in self.controller_joint_names_dict.keys():
                if ("gripper" in name):
                    continue
                point = self.GetJointTrajectoryPoint(name)
                if point is None:
                    continue
                teaching_point_manager.RecordTeachingPointToRosParam(self.nameSpace + name, tpName, point)
            self._widget.spinBoxTpNumber.setValue(tpNum+1)
        elif sender == self._widget.buttonMoveTp:
            self.MoveTeachingPoint(tpName, transitionTime)
            self._widget.spinBoxTpNumber.setValue(tpNum+1)
        elif sender == self._widget.buttonGoHome:
            self.GoHome(transitionTime)
        elif sender == self._widget.buttonTrajRecord:
            if self._widget.buttonTrajRecord.text() == "Record\nStart":
                self._widget.buttonTrajRecord.setText("Record\nStop")
                self._widget.buttonTrajRun.setEnabled(False)
                self._widget.buttonTeachingMode.click()
                self.StartTrajectoryRecord(recordInterval)
            else:
                self.StopTrajectoryRecord()
                self.RecordTrajectory(trajName)
                self._widget.buttonTrajRecord.setText("Record\nStart")
                self._widget.buttonTrajRun.setEnabled(True)
        elif sender == self._widget.buttonTrajRun:
            self.MoveTeachingTrajectory(trajName)
        elif sender == self._widget.buttonSaveRosParam:
            self.SaveRosParam()
        elif sender == self._widget.buttonLoadRosParam:
            self.LoadRosParam()

    def GoHome(self, transitionTime):
        if(self.goHomeActionServerConnection == False):
            # pre-connect to action server for avoiding delay
            ret = True
            for name in self.controller_joint_names_dict.keys():
                if ("gripper" in name):
                    continue
                ret &= home_position_manager.connect_server(self.nameSpace + name, timeout=1.0)
            if(ret):
                self.goHomeActionServerConnection = True

        self._widget.buttonMovingMode.click()
        for name in self.controller_joint_names_dict.keys():
            if ("gripper" in name):
                continue
            home_position_manager.MoveToHomePosition(self.nameSpace + name, transitionTime, timeout=0.5)

    def MoveTeachingPoint(self, tpName, transitionTime):
        if(self.moveTeachingPointActionServerConnection == False):
            # pre-connect to action server for avoiding delay
            ret = True
            for name in self.controller_joint_names_dict.keys():
                if ("gripper" in name):
                    continue
                ret &= teaching_point_manager.connect_server(self.nameSpace + name, timeout=1.0)
            if(ret):
                self.moveTeachingPointActionServerConnection = True
        for name in self.controller_joint_names_dict.keys():
            if ("gripper" in name):
                continue
            teaching_point_manager.MoveToTeachingPoint(self.nameSpace + name, tpName, transitionTime, timeout=0.5)

    def MoveTeachingTrajectory(self, trajName):
        if(self.moveTeachingTrajectoryActionServerConnection == False):
            # pre-connect to action server for avoiding delay
            ret = True
            for name in self.controller_joint_names_dict.keys():
                if ("gripper" in name):
                    continue
                ret &= teaching_trajectory_manager.connect_server(self.nameSpace + name, timeout=1.0)
            if(ret):
                self.moveTeachingTrajectoryActionServerConnection = True
        for name in self.controller_joint_names_dict.keys():
            if ("gripper" in name):
                continue
            teaching_trajectory_manager.MoveToTeachingTrajectory(self.nameSpace + name, trajName, timeout=0.5)

    def StartTrajectoryRecord(self, recordInterval, startTimer=True):
        print "Record Start"
        self.recordPoints = {}
        for name in self.controller_joint_names_dict.keys():
            if ("gripper" in name):
                continue
            self.recordPoints[name] = []
        self.recordPointsNum = 0
        self.recordIntervalMS = recordInterval * 1000.0
        if startTimer:
            self.recordTimer.start(self.recordIntervalMS)

    def StopTrajectoryRecord(self):
        print "Record Stop"
        self.recordTimer.stop()
    
    def RecordTrajectory(self, trajName):
        for name in self.controller_joint_names_dict.keys():
            if ("gripper" in name):
                continue
            traj = JointTrajectory()
            traj.joint_names = self.toroboJointStateJointNames[name]
            traj.points = self.recordPoints[name]
            teaching_trajectory_manager.RecordTeachingTrajectoryToRosParam(self.nameSpace + name, trajName, traj)

    def IncrementRecordTime(self):
        self.recordTime = numpy.round(self.recordIntervalMS * self.recordPointsNum / 1000.0, 2)
    
    def GetJointTrajectoryPoint(self, controllerName):
        toroboJointState = self.toroboJointState[controllerName]
        if(toroboJointState is None):
            return None
        self.toroboJointStateJointNames[controllerName] = toroboJointState.name
        point = JointTrajectoryPoint()
        point.positions = toroboJointState.position
        point.velocities = toroboJointState.velocity
        point.accelerations = toroboJointState.acceleration
        point.effort = toroboJointState.effort
        return point

    def RecordOnePoint(self):
        self.IncrementRecordTime()
        for name in self.controller_joint_names_dict.keys():
            if ("gripper" in name):
                continue
            point = self.GetJointTrajectoryPoint(name)
            if(point is None):
                continue
            point.time_from_start = rospy.Duration.from_sec(self.recordTime)
            self.recordPoints[name].append(point)
        self.recordPointsNum += 1

    def TrajectoryRecordCallback(self):
        self.RecordOnePoint()
        self._widget.lcdNumberRecordTime.display(str(self.recordTime))
        self._widget.lcdNumberRecordedPoints.display(str(self.recordPointsNum))

    def SaveRosParam(self):
        loadfile = QFileDialog.getSaveFileName(self._widget, "Save rosparam", "~/torobo_teaching_param.yaml", "Yaml (*.yaml)")
        fileName = loadfile[0]
        if fileName != "":
            param = {}
            for name in self.controller_joint_names_dict.keys():
                if ("gripper" in name):
                    continue
                param[name] = {}
                tps = rospy.get_param(self.nameSpace + name + "/teaching_points", None)
                trajs = rospy.get_param(self.nameSpace + name + "/teaching_trajectories", None)
                if (tps != None):
                    param[name]['teaching_points'] = tps
                if (trajs != None):
                    param[name]['teaching_trajectories'] = trajs

            with open(fileName, "w") as outfile:
                yaml.dump(param, outfile, default_flow_style=False)
            rospy.loginfo("Saved rosparam to [%s]" % fileName)
        else:
            rospy.loginfo("Not saved")

    def LoadRosParam(self):
        loadfile = QFileDialog.getOpenFileName(self._widget, "Please select loading rosparam yaml file", "~/", "Yaml (*.yaml)")
        fileName = loadfile[0]
        if(os.path.exists(fileName) == False):
            rospy.loginfo("[%s] is invalid rosparam file." % fileName)
            return
        paramlist = rosparam.load_file(fileName, self.nameSpace)
        for params, ns in paramlist:
            rosparam.upload_params(ns, params)
        rospy.loginfo("Loaded rosparam from [%s]" % fileName)

    def OpenSaveFileDialog(self):
        filename = QFileDialog.getSaveFileName(self._widget, "Save rosparam", "~/torobo_teaching_param.yaml", "Yaml (*.yaml)")
        return filename

class eSystemMode:
    STOP = 0
    READY = 1
    RUN = 2
    ERROR = 3

class eHighCtrlMode:
    TRAJ = 0
    EXTERNAL_FORCE_FOLLOWING = 5