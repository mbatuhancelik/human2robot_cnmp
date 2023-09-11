#!/usr/bin/env python
## -*- coding: utf-8 -*-
import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *

import numpy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Header

from torobo_motion_manager import home_position_manager
from torobo_motion_manager import teaching_point_manager
from torobo_motion_manager import teaching_trajectory_manager

import torobo_common.rosparam_loader as rosparam_loader

class ToroboTeachingManager(Plugin):
    def __init__(self, context):
        super(self.__class__, self).__init__(context)
        self.setObjectName('ToroboTeachingManager')

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
        ui_file = os.path.join(rospkg.RosPack().get_path('torobo_gui'), 'resource', 'torobo_teaching_manager.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ToroboTeachingManagerUi')

        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self.serial_number = context.serial_number()
        ns = rospy.get_namespace()
        self.create(ns)

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

    def create(self, nameSpace):
        self.controller_joint_names_dict = rosparam_loader.get_controller_name_joint_names_dict("controller_list", sorting=True)

        self.initMainForm(nameSpace)
        self.initTpTab()
        self.initTrajectoryTab()

        self.sub = None
        self.nameDict = {}
        self.jointTrajectoryPoint = JointTrajectoryPoint()

        self.recordTimer = QTimer(self)
        self.recordTimer.timeout.connect(self.TrajectoryRecordCallback)

        if self._widget.comboBoxControllerName.currentIndex() != -1:
            self._widget.pushButtonConnect.click()

    def destroy(self):
        self.recordTimer.stop()
        self.UnregisterSubscriber()

    def setup(self, controllerName):
        self.ClearTpTable()
        self.ClearTrajectoryTable()
        self.controllerName = controllerName
        self.controllerNameSpace = controllerName + "/"
        self.jointsNum = len(self.controller_joint_names_dict[controllerName])
        self.updateForm(self.jointsNum)
        self.UnregisterSubscriber()
        self.RegisterSubscriber(self.controllerName)

    def GetControllerNames(self):
        names = []
        for (k, v) in self.controller_joint_names_dict.items():
            if("gripper" in k):
                continue
            names.append(k)
        return names

    def initMainForm(self, ns):
        # NameSpace
        self._widget.comboBoxControllerNamespace.addItem(ns)
        names = self.GetControllerNames()
        for name in names:
            self._widget.comboBoxControllerName.addItem(name)
        idx = 0
        if (self.serial_number <= self._widget.comboBoxControllerName.count()):
            idx = self.serial_number-1
            while True:
                print self._widget.comboBoxControllerName.itemText(idx)
                if "gripper" not in self._widget.comboBoxControllerName.itemText(idx):
                    break
                elif idx < self._widget.comboBoxControllerName.count():
                    idx += 1
                else:
                    break
                
        self._widget.comboBoxControllerName.setCurrentIndex(idx)

        # button
        self._widget.pushButtonConnect.clicked.connect(self.button_clicked)

    def initTpTab(self):
        # row
        tableTpVertHeaders=()
        for i in range(100):
            tableTpVertHeaders += (str(i),)
        self._widget.tableTp.setRowCount(100)
        self._widget.tableTp.setVerticalHeaderLabels( tableTpVertHeaders )

        # connect
        self._widget.tableTp.cellClicked.connect(self.cell_clicked)
        self._widget.buttonGetTp.clicked.connect(self.button_clicked)
        self._widget.buttonMoveTp.clicked.connect(self.button_clicked)
        self._widget.buttonClearTp.clicked.connect(self.button_clicked)
        self._widget.buttonSaveTp.clicked.connect(self.button_clicked)
        self._widget.buttonLoadTp.clicked.connect(self.button_clicked)
        self._widget.buttonGoHome.clicked.connect(self.button_clicked)

    def updateTpTable(self, jointsNum):
        # column
        self._widget.tableTp.setColumnCount(jointsNum+1)
        horzHeaders = (
            "Name",
        )
        self._widget.tableTp.setColumnWidth(0, 65)
        for i in range(jointsNum):
            horzHeaders += ("J" + str(i+1) + " Pos",)
            self._widget.tableTp.setColumnWidth(i+1, 65)
        self._widget.tableTp.setHorizontalHeaderLabels( horzHeaders )

        # Load teaching points
        self.LoadTeachingPointsFromRosParam()

    def initTrajectoryTab(self):
        # row
        tableTrajVertHeaders=()
        for i in range(1000):
            tableTrajVertHeaders += (str(i),)
        self._widget.tableTrajectory.setRowCount(1000)
        self._widget.tableTrajectory.setVerticalHeaderLabels( tableTrajVertHeaders )

        # button
        self._widget.buttonTrajRecord.clicked.connect(self.button_clicked)
        self._widget.buttonTrajRun.clicked.connect(self.button_clicked)
        self._widget.buttonClearTraj.clicked.connect(self.button_clicked)
        self._widget.buttonGoHomeTrajTab.clicked.connect(self.button_clicked)
        self._widget.buttonSaveTraj.clicked.connect(self.button_clicked)
        self._widget.buttonLoadTraj.clicked.connect(self.button_clicked)

    def updateTrajectoryTable(self, jointsNum):
        # column
        self._widget.tableTrajectory.setColumnCount(jointsNum*2+1)
        horzHeaders = (
            "Time",
        )
        self._widget.tableTrajectory.setColumnWidth(0, 65)
        for i in range(jointsNum):
            horzHeaders += ("J" + str(i+1) + " Pos",)
            horzHeaders += ("J" + str(i+1) + " Vel",)
            self._widget.tableTrajectory.setColumnWidth(2*i+1, 65)
            self._widget.tableTrajectory.setColumnWidth(2*i+2, 65)
        self._widget.tableTrajectory.setHorizontalHeaderLabels( horzHeaders )

        # Load teaching trajectory
        trajName = unicode(self._widget.lineEditTrajName.text())
        self.LoadTeachingTrajectoryFromRosParam(trajName)

    def updateForm(self, jointsNum):
        self.updateTpTable(jointsNum)
        self.updateTrajectoryTable(jointsNum)

    def cell_clicked(self):
        row = self._widget.tableTp.currentRow()
        self._widget.spinBoxTpNumber.setValue(row)

    def button_clicked(self):
        sender = self.sender()
        if sender == self._widget.pushButtonConnect:
            if (self._widget.pushButtonConnect.text() == "Connect"):
                controllerName = self._widget.comboBoxControllerName.currentText()
                self.setup(controllerName)
                self._widget.pushButtonConnect.setText("Disconnect")
                self._widget.comboBoxControllerNamespace.setEnabled(False)
                self._widget.comboBoxControllerName.setEnabled(False)
            else:
                self.UnregisterSubscriber()
                self._widget.pushButtonConnect.setText("Connect")
                self._widget.comboBoxControllerNamespace.setEnabled(True)
                self._widget.comboBoxControllerName.setEnabled(True)

        if (self._widget.pushButtonConnect.text() == "Connect"):
            return

        if sender == self._widget.buttonClearTp:
            self.ClearTpTable()
        elif sender == self._widget.buttonGetTp:
            tpNum = int(self._widget.spinBoxTpNumber.text())
            item = self._widget.tableTp.item(tpNum, 0)
            name = "tp" + str(tpNum)
            if item is not None:
                name = unicode(item.text())
            point = self.jointTrajectoryPoint
            self.SetTeachingPointToTable(tpNum, name, point.positions)
            teaching_point_manager.RecordTeachingPointToRosParam(self.controllerNameSpace, name, point)
            self._widget.spinBoxTpNumber.setValue(tpNum+1)
        elif sender == self._widget.buttonMoveTp:
            tpNum = int(self._widget.spinBoxTpNumber.text())
            transitionTime = float(self._widget.doubleSpinBoxTransitionTime.text())
            self.SendTeachingPoint(tpNum, transitionTime)
            self._widget.spinBoxTpNumber.setValue(tpNum+1)
        elif sender == self._widget.buttonSaveTp:
            self.SaveTeachingPointsToRosParam()
        elif sender == self._widget.buttonLoadTp:
            self.ClearTpTable()
            self.LoadTeachingPointsFromRosParam()
        elif (sender == self._widget.buttonGoHome) or (sender == self._widget.buttonGoHomeTrajTab):
            transitionTime = float(self._widget.doubleSpinBoxTransitionTime.text())
            home_position_manager.MoveToHomePosition(self.controllerNameSpace, transitionTime)
        elif sender == self._widget.buttonClearTraj:
            self.ClearTrajectoryTable()
        elif sender == self._widget.buttonTrajRecord:
            if (self._widget.buttonTrajRecord.text() == "Record\nStart"):
                trajName = unicode(self._widget.lineEditTrajName.text())
                recordInterval = float(self._widget.doubleSpinBoxRecordInterval.text())
                self.StartTrajectoryRecord(recordInterval)
                self._widget.buttonTrajRecord.setText("Record\nStop")
                self._widget.buttonTrajRun.setEnabled(False)
            else:
                self.StopTrajectoryRecord()
                self._widget.buttonTrajRecord.setText("Record\nStart")
                self._widget.buttonTrajRun.setEnabled(True)
                self._widget.buttonSaveTraj.click()
        elif sender == self._widget.buttonTrajRun:
            trajName = unicode(self._widget.lineEditTrajName.text())
            self.SendTeachingTrajectory(trajName)
        elif sender == self._widget.buttonSaveTraj:
            trajName = unicode(self._widget.lineEditTrajName.text())
            self.SaveTeachingTrajectoryToRosParam(trajName)
        elif sender == self._widget.buttonLoadTraj:
            trajName = unicode(self._widget.lineEditTrajName.text())
            traj = self.LoadTeachingTrajectoryFromRosParam(trajName)

    def ClearTpTable(self):
        for i in range(self._widget.tableTp.rowCount()):
            for j in range(self._widget.tableTp.columnCount()):
                self._widget.tableTp.setItem(i, j, None)

    def ClearTrajectoryTable(self):
        for i in range(self._widget.tableTrajectory.rowCount()):
            for j in range(self._widget.tableTrajectory.columnCount()):
                self._widget.tableTrajectory.setItem(i, j, None)

    def TrajectoryRecordCallback(self):
        self.RecordOnePoint()

    def SetTeachingPointToTable(self, row, tpName, positions):
        self._widget.tableTp.setItem(row, 0, QTableWidgetItem(tpName))
        for i in range(len(positions)):
            pos = numpy.round(numpy.degrees(positions[i]),3)
            self._widget.tableTp.setItem(row, i+1, QTableWidgetItem(str(pos)))

    def ImportTeachingPointFromTable(self, tpNum):
        positions = []
        item = self._widget.tableTp.item(tpNum, 0)
        if item is None:
            return None, positions
        name = unicode(item.text())
        for i in range(self._widget.tableTp.columnCount()-1):
            item = self._widget.tableTp.item(tpNum, i+1)
            if item is not None:
                pos = float(item.text())
                positions.append(pos)
        return name, positions

    def SendTeachingPoint(self, tpNum, transitionTime):
        [name, positions] = self.ImportTeachingPointFromTable(tpNum)
        if (name is None):
            print "Invalid Teaching Point."
            return
        positions = map(numpy.radians, positions)

        print "Send TP: ", positions
        point = teaching_point_manager.ConvertPositionsToJointTrajectoryPoint(positions)
        teaching_point_manager.RecordTeachingPointToRosParam(self.controllerNameSpace, name, point)
        teaching_point_manager.MoveToTeachingPoint(self.controllerNameSpace, name, transitionTime)
    
    def LoadTeachingPointsFromRosParam(self):
        names = teaching_point_manager.GetTeachingPointNamesFromRosParam(self.controllerNameSpace)
        if names is None:
            return
        tpLen = 0
        for (i, name) in enumerate(names):
            point = teaching_point_manager.GetTeachingPointFromRosParam(self.controllerNameSpace, name)
            if point != None:
                tpLen += 1
                self.SetTeachingPointToTable(i, name, point.positions)
        print "Load [" + str(tpLen) + "] teaching points from rosparam [" + self.controllerNameSpace + "]"

    def SaveTeachingPointsToRosParam(self):
        tpLen = 0
        self.DeleteAllTeachingPointFromRosParam()
        for i in range(self._widget.tableTp.rowCount()):
            [name, positions] = self.ImportTeachingPointFromTable(i)
            if name is not None:
                positions = map(numpy.radians, positions)
                point = teaching_point_manager.ConvertPositionsToJointTrajectoryPoint(positions)
                teaching_point_manager.RecordTeachingPointToRosParam(self.controllerNameSpace, name,  point)
                tpLen += 1
        print "Save [" + str(tpLen) + "] teaching points to rosparam [" + self.controllerNameSpace + "]"

    def DeleteAllTeachingPointFromRosParam(self):
        names = teaching_point_manager.GetTeachingPointNamesFromRosParam(self.controllerNameSpace)
        if names == None:
            return
        for name in names:
            teaching_point_manager.DeleteTeachingPointFromRosParam(self.controllerNameSpace, name)

    def DeleteTeachingPointFromRosParam(self, tpName):
        teaching_point_manager.DeleteTeachingPointFromRosParam(self.controllerNameSpace, tpName)

    def SetTrajectoryPointToTable(self, row, time, point):
        self._widget.tableTrajectory.setItem(row, 0, QTableWidgetItem(str(time)))
        for i in range(len(point.positions)):
            pos = numpy.round(numpy.degrees(point.positions[i]), 3)
            vel = numpy.round(numpy.degrees(point.velocities[i]), 3)
            self._widget.tableTrajectory.setItem(row, 2*i+1, QTableWidgetItem(str(pos)))
            self._widget.tableTrajectory.setItem(row, 2*i+2, QTableWidgetItem(str(vel)))

    def SetTeachingTrajectoryToTable(self, traj):
        self.ClearTrajectoryTable()
        for i, p in enumerate(traj.points):
            time = p.time_from_start.to_sec()
            self.SetTrajectoryPointToTable(i, time, p)

    def ImportTeachingTrajectoryFromTable(self):
        points = []
        for i in range(self._widget.tableTrajectory.rowCount()):
            point = None
            for j in range(self._widget.tableTrajectory.columnCount()):
                item = self._widget.tableTrajectory.item(i, j)
                if item is None:
                    break
                if j == 0:
                    duration = float(item.text())
                    point = JointTrajectoryPoint()
                    point.time_from_start = rospy.Duration.from_sec(duration)
                elif j % 2 == 1:
                    pos = float(item.text())
                    point.positions.append(numpy.radians(pos))
                else:
                    vel = float(item.text())
                    point.velocities.append(numpy.radians(vel))
                    point.accelerations.append(0.0)
                    point.effort.append(0.0)
            if point != None:
                points.append(point)
        
        if points == []:
            return None

        traj = JointTrajectory()
        for i in range(len(points[0].positions)):
            traj.joint_names.append("joint_" + str(i+1))
        traj.points = points

        return traj

    def StartTrajectoryRecord(self, recordInterval, startTimer=True):
        print "Record Start"
        self.ClearTrajectoryTable()
        self.recordPointsNum = 0
        self.recordIntervalMS = recordInterval * 1000.0
        if startTimer:
            self.recordTimer.start(self.recordIntervalMS)

    def StopTrajectoryRecord(self):
        print "Record Stop"
        self.recordTimer.stop()

    def IncrementRecordTime(self):
        self.recordTime = numpy.round(self.recordIntervalMS * self.recordPointsNum / 1000.0, 2)

    def RecordOnePoint(self):
        self.IncrementRecordTime()
        self.SetTrajectoryPointToTable(self.recordPointsNum, self.recordTime, self.jointTrajectoryPoint)
        self._widget.tableTrajectory.selectRow(self.recordPointsNum)
        self.recordPointsNum += 1

    def SendTeachingTrajectory(self, trajName):
        traj = self.ImportTeachingTrajectoryFromTable()
        if (traj is None):
            print "Invalid Teaching Trajecotry."
            return

        print "Send Teaching Trajectory"
        teaching_trajectory_manager.RecordTeachingTrajectoryToRosParam(self.controllerNameSpace, trajName, traj)
        teaching_trajectory_manager.MoveToTeachingTrajectory(self.controllerNameSpace, trajName)

    def LoadTeachingTrajectoryFromRosParam(self, trajNum):
        name = unicode(self._widget.lineEditTrajName.text())
        traj = teaching_trajectory_manager.GetTeachingTrajectoryFromRosParam(self.controllerNameSpace, name)
        if traj != None:
            self.SetTeachingTrajectoryToTable(traj)
        return traj
    
    def SaveTeachingTrajectoryToRosParam(self, trajName):
        traj = self.ImportTeachingTrajectoryFromTable()
        if (traj is None):
            return
        teaching_trajectory_manager.RecordTeachingTrajectoryToRosParam(self.controllerNameSpace, trajName, traj)

    def UnregisterSubscriber(self):
        if self.sub != None:
            self.sub.unregister()

    def RegisterSubscriber(self, controllerName):
        ns =         self.controllerNameSpace + 'state'
        self.sub = rospy.Subscriber(ns, JointTrajectoryControllerState, self.callback, queue_size=1)

    def callback(self, state):
        self.jointTrajectoryPoint = state.actual
