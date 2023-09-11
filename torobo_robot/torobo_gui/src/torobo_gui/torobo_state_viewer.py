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
from collections import OrderedDict
import torobo_driver.torobo_easy_command
import torobo_common.rosparam_loader as rosparam_loader
from torobo_msgs.msg import ToroboJointState

class ToroboStateViewer(Plugin):
    def __init__(self, context):
        super(self.__class__, self).__init__(context)
        self.setObjectName('ToroboStateViewer')

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
        ui_file = os.path.join(rospkg.RosPack().get_path('torobo_gui'), 'resource', 'torobo_state_viewer.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ToroboStateViewerUi')

        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self.serial_number = context.serial_number()
        self.create(100)

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
    
    def create(self, updateTimeInterval):
        self.configNamespace = "torobo_controller_config/"
        self.initMainForm()
        self.initTable()
        self.initTableSub()
        self.initTableEtc()

        self.sub = {}
        self.sub_time = {}
        self.nameDict = {}
        self.robot = {}
        self.robot['stamp'] = 0
        self.robot['timeStamp'] = 0
        self.robot['hostTimeStamp'] = 0
        self.updateTimeInterval = updateTimeInterval
        self.updateTimer = QTimer(self)
        self.updateTimer.timeout.connect(self.update)
        self.updateTimer.start(updateTimeInterval)

        self.recoveryTargetJoint = []
        self.recoveryTargetJointStartPos = {}
        self.timer_recovery_over_position = QTimer(self)
        self.timer_recovery_over_position.timeout.connect(self.RecoveryOverPositionCallback)
        self.timer_recovery_over_position.start(500)

        if self._widget.comboBoxControllerName.currentIndex() != -1:
            self._widget.pushButtonConnect.click()
    
    def destroy(self):
        self.updateTimer.stop()
        self.UnregisterSubscriber()

    def setup(self, configNamespace, controllerName):
        self.controllerList = rosparam_loader.get_controller_list_from_torobo_controller_config(controllerName, configNamespace, sorting=False)
        self.controller_joint_names_dict = self.GetControllerJointNameDict(self.controllerList)
        self.controller_joint_ids_dict = self.GetControllerJointIdDict(self.controllerList)
        self.nameDict = self.GetJointNameIdDict(self.controllerList)
        self.jointsNum = len(self.nameDict)
        self.updateForm(self.jointsNum)
        self.UnregisterSubscriber()
        self.RegisterSubscriber(self.controllerList)

    def GetControllerJointNameDict(self, controller_list):
        dic = OrderedDict()
        for controller in controller_list:
            joint_names = rosparam_loader.get_joint_names(controller)
            if(joint_names == []):
                continue
            dic[controller] = rosparam_loader.get_joint_names(controller)
        return dic

    def GetControllerJointIdDict(self, controller_list):
        dic = OrderedDict()
        id = 0
        for controller in controller_list:
            joint_names = rosparam_loader.get_joint_names(controller)
            l = []
            for joint_name in joint_names:
                l.append(id)
                id += 1
            dic[controller] = l
        return dic

    def GetJointNameIdDict(self, controller_list):
        dic = OrderedDict()
        id = 0
        for controller in controller_list:
            joint_names = rosparam_loader.get_joint_names(controller)
            for joint_name in joint_names:
                dic[joint_name] = id
                id += 1
        return dic

    def initMainForm(self):
        # NameSpace
        self._widget.comboBoxControllerConfigNamespace.addItem(self.configNamespace)
        names = rosparam_loader.get_torobo_controller_names(self.configNamespace)
        for name in names:
            self._widget.comboBoxControllerName.addItem(name)
        if (self.serial_number <= self._widget.comboBoxControllerName.count()):
            self._widget.comboBoxControllerName.setCurrentIndex(self.serial_number-1)

        # button
        self._widget.pushButtonConnect.clicked.connect(self.button_clicked)
        self._widget.pushButtonCommandSend.clicked.connect(self.button_clicked)
        self._widget.lineEditCommand.returnPressed.connect(self.lineEdit_return_pressed)
        self._widget.pushButtonServoOn.clicked.connect(self.button_clicked)
        self._widget.pushButtonServoOff.clicked.connect(self.button_clicked)
        self._widget.pushButtonTrajMode.clicked.connect(self.button_clicked)
        self._widget.pushButtonExtFFMode.clicked.connect(self.button_clicked)
        self._widget.pushButtonErrorReset.clicked.connect(self.button_clicked)
        self._widget.pushButtonGoHome.clicked.connect(self.button_clicked)

    def initTable(self):
        horzHeaders = (
            "State",
            "Mode",
            "Pos",
            "Vel",
            "Cur",
            "Eft",
            "Error",
        )
        self._widget.table.setColumnCount(len(horzHeaders))
        self._widget.table.setHorizontalHeaderLabels( horzHeaders )
        self._widget.table.setColumnWidth(0, 70)
        self._widget.table.setColumnWidth(1, 60)
        self._widget.table.setColumnWidth(2, 60)
        self._widget.table.setColumnWidth(3, 60)
        self._widget.table.setColumnWidth(4, 60)
        self._widget.table.setColumnWidth(5, 60)
        self._widget.table.setColumnWidth(6, 100)
        self._widget.table.horizontalHeader().setStretchLastSection(True)
        
    def initTableSub(self):
        horzHeaders = (
            "State",
            "Type",
            "Temp",
            "Kp",
            "Ki",
            "Kd",
            "Windup",
            "Via",
            "TrajStatus",
        )
        self._widget.tableSub.setColumnCount(len(horzHeaders))
        self._widget.tableSub.setHorizontalHeaderLabels( horzHeaders )
        self._widget.tableSub.setColumnWidth(0, 70)
        self._widget.tableSub.setColumnWidth(1, 50)
        self._widget.tableSub.setColumnWidth(2, 50)
        self._widget.tableSub.setColumnWidth(3, 60)
        self._widget.tableSub.setColumnWidth(4, 60)
        self._widget.tableSub.setColumnWidth(5, 60)
        self._widget.tableSub.setColumnWidth(6, 60)
        self._widget.tableSub.setColumnWidth(7, 40)
        self._widget.tableSub.setColumnWidth(8, 110)
        self._widget.tableSub.horizontalHeader().setStretchLastSection(True)
    
    def initTableEtc(self):
        self.button_recoverOverPos = {}
        self._widget.tableEtc.setColumnCount(1)

    def updateTableEtc(self, jointsNum):
        rows = self._widget.tableEtc.rowCount()
        cols = self._widget.tableEtc.columnCount()
        for i in range(rows):
            for j in range(cols):
                self._widget.tableEtc.removeCellWidget(i, j)
        for (k, v) in self.button_recoverOverPos.items():
            v.clicked.disconnect()

        self._widget.tableEtc.setRowCount(jointsNum)
        self.button_recoverOverPos = {} 
        for (k, v) in self.nameDict.items():
            button_recoverOverPos = QPushButton("BrakeOff")
            widget = QWidget()
            layout = QHBoxLayout(widget)
            layout.addWidget(button_recoverOverPos)
            layout.setAlignment(Qt.AlignCenter)
            layout.setContentsMargins(0, 0, 0, 0)
            widget.setLayout(layout)
            button_recoverOverPos.setObjectName(k)
            button_recoverOverPos.clicked.connect(self.button_recover_overpos_clicked)
            self.button_recoverOverPos[k] = button_recoverOverPos
            self._widget.tableEtc.setCellWidget(v, 0, widget)

    def updateForm(self, jointsNum):
        self._widget.table.setRowCount(jointsNum)
        self._widget.tableSub.setRowCount(jointsNum)
        self.updateTableEtc(jointsNum)

    def lineEdit_return_pressed(self):
        sender = self.sender()
        if sender == self._widget.lineEditCommand:
            text = self._widget.lineEditCommand.text()
            self._widget.lineEditCommand.setText("")
            try:
                text = str(text)
            except:
                return
            self.send_command(text)

    def button_clicked(self):
        sender = self.sender()
        if sender == self._widget.pushButtonConnect:
            if (self._widget.pushButtonConnect.text() == "Connect"):
                controllerName = self._widget.comboBoxControllerName.currentText()
                self.setup(self.configNamespace, controllerName)
                self._widget.pushButtonConnect.setText("Disconnect")
            else:
                self.UnregisterSubscriber()
                self._widget.pushButtonConnect.setText("Connect")
        elif sender == self._widget.pushButtonCommandSend:
            text = self._widget.lineEditCommand.text()
            self._widget.lineEditCommand.setText("")
            try:
                text = str(text)
            except:
                return
            self.send_command(text)
        elif sender == self._widget.pushButtonServoOn:
            self.send_command("s all 1")
        elif sender == self._widget.pushButtonServoOff:
            self.send_command("s all 0")
        elif sender == self._widget.pushButtonTrajMode:
            self.send_command("r all 0")
        elif sender == self._widget.pushButtonExtFFMode:
            self.send_command("r all 5")
        elif sender == self._widget.pushButtonErrorReset:
            self.send_command("reset all")
        elif sender == self._widget.pushButtonGoHome:
            self.send_command("r all 0")
            self.send_command("home all 5")

    def button_recover_overpos_clicked(self):
        sender = self.sender()
        name = sender.objectName()
        if name in self.nameDict:
            self.StartRecoveryOverPositionSequence(name)

    def send_command(self, text):
        sp = text.split(" ")
        if len(sp) < 2:
            return
        tag = sp[0]
        cmd = []
        for i in range(len(sp)-1):
            cmd.append(sp[i+1])

        try:
            [joint_name, whole_order, joint_order, value1, value2, value3, value4] = torobo_driver.torobo_easy_command.ParseMasterCommand(tag, cmd)
        except Exception, e:
            rospy.loginfo("input: %s" % text)
            rospy.loginfo(e)
            return
        found = False
        for (controller_name, joint_names) in self.controller_joint_names_dict.items():
            for j in joint_name: 
                if (j == "all") or (j in joint_names):
                    found = True
                    torobo_driver.torobo_easy_command.SendEasyCommand(controller_name, tag, cmd)
                    continue
                elif (j.isdigit()) and ((int(j)-1) in self.controller_joint_ids_dict[controller_name]):
                    found = True
                    torobo_driver.torobo_easy_command.SendEasyCommand(controller_name, tag, cmd)
                    continue
        if not found:
            rospy.loginfo("input: %s" % text)
            rospy.loginfo("invalid joint name")

    def update(self):
        self._widget.lineEditTimestamp.setText(str(self.robot['timeStamp']))
        self._widget.lineEditHostTimestamp.setText(str(self.robot['hostTimeStamp']))

        for name, id in self.nameDict.items():
            if not name in self.robot:
                continue
            coef = 1.0
            if (self.robot[name]['type'] == eJointType.JOINT_TYPE_GRIPPER_SLIDE):
                coef = 1000.0
            else:
                coef = 180.0 / numpy.pi

            # decide 'status' value
            com = self.robot[name]['comStatus']
            comStatus = self.comStatus_to_item(com)
            systemMode = self.systemMode_to_item(self.robot[name]['systemMode'])
            status = systemMode
            if(com == eComStatus.TIMEOUT_ERROR or com == eComStatus.CRC_ERROR):
                status = comStatus

            if(self._widget.tabWidget.currentIndex() == 0):
                self._widget.table.setItem(id,0, status)
                self._widget.table.setItem(id,1, QTableWidgetItem(self.ctrlMode_to_str(self.robot[name]['ctrlMode'])))
                self._widget.table.setItem(id,2, QTableWidgetItem(str(round(self.robot[name]['position'] * coef, 4))))
                self._widget.table.setItem(id,3, QTableWidgetItem(str(round(self.robot[name]['velocity'] * coef, 4))))
                self._widget.table.setItem(id,4, QTableWidgetItem(str(round(self.robot[name]['current'], 4))))
                self._widget.table.setItem(id,5, QTableWidgetItem(str(round(self.robot[name]['effort'], 4))))
                self._widget.table.setItem(id,6, self.ewStatus_to_item(self.robot[name]['errorWarningStatus']))
            else:
                self._widget.tableSub.setItem(id,0, status)
                self._widget.tableSub.setItem(id,1, QTableWidgetItem(self.jointType_to_str(self.robot[name]['type'])))
                self._widget.tableSub.setItem(id,2, QTableWidgetItem(str(round(self.robot[name]['temperature'], 3))))
                self._widget.tableSub.setItem(id,3, QTableWidgetItem(str(round(self.robot[name]['general_0'], 5))))
                self._widget.tableSub.setItem(id,4, QTableWidgetItem(str(round(self.robot[name]['general_1'], 5))))
                self._widget.tableSub.setItem(id,5, QTableWidgetItem(str(round(self.robot[name]['general_2'], 5))))
                self._widget.tableSub.setItem(id,6, QTableWidgetItem(str(round(self.robot[name]['general_3'], 5))))
                self._widget.tableSub.setItem(id,7, QTableWidgetItem(str(self.robot[name]['trjViaRemain'])))
                self._widget.tableSub.setItem(id,8, QTableWidgetItem(self.trjStatus_to_str(self.robot[name]['trjStatus'])))
    
    def UnregisterSubscriber(self):
        for (k, v) in self.sub.items():
            v.unregister()
        self.sub_time = {}

    def RegisterSubscriber(self, controllerList):
        for k in controllerList:
            self.sub[k] = rospy.Subscriber('joint_state_server/' + k + '/torobo_joint_state', ToroboJointState, self.callback, callback_args=k, queue_size=1)
            self.sub_time[k] = rospy.get_time()

    def callback(self, toroboJointState, name):
        t = rospy.get_time()
        dt = t - self.sub_time[name]
        if (dt < self.updateTimeInterval / 1000.0):
            return
        self.sub_time[name] = t

        self.robot['stamp'] = toroboJointState.header.stamp
        self.robot['timeStamp'] = toroboJointState.timeStamp
        self.robot['hostTimeStamp'] = toroboJointState.hostTimeStamp

        for name in toroboJointState.name:
            if not name in self.robot:
                self.robot[name] = {}
            id = toroboJointState.name.index(name)
            self.robot[name]['type'] = toroboJointState.type[id]
            self.robot[name]['comStatus'] = toroboJointState.comStatus[id]
            self.robot[name]['systemMode'] = toroboJointState.systemMode[id]
            self.robot[name]['ctrlMode'] = toroboJointState.ctrlMode[id]
            self.robot[name]['errorWarningStatus'] = toroboJointState.errorWarningStatus[id]
            self.robot[name]['trjStatus'] = toroboJointState.trjStatus[id]
            self.robot[name]['trjViaRemain'] = toroboJointState.trjViaRemain[id]
            self.robot[name]['current'] = toroboJointState.current[id]
            self.robot[name]['position'] = toroboJointState.position[id]
            self.robot[name]['velocity'] = toroboJointState.velocity[id]
            self.robot[name]['acceleration'] = toroboJointState.acceleration[id]
            self.robot[name]['effort'] = toroboJointState.effort[id]
            self.robot[name]['temperature'] = toroboJointState.temperature[id]
            self.robot[name]['general_0'] = toroboJointState.general_0[id]
            self.robot[name]['general_1'] = toroboJointState.general_1[id]
            self.robot[name]['general_2'] = toroboJointState.general_2[id]
            self.robot[name]['general_3'] = toroboJointState.general_3[id]

    def IsOverPosition(self, targetJoint):
        if not targetJoint in self.robot:
            return False
        value = self.robot[targetJoint]["errorWarningStatus"]
        if (value & (1 << eMasterErrorWarningStatus.ERROR_OVER_POSITION)) != 0:
            return True
        return False

    def StartRecoveryOverPositionSequence(self, targetJoint):
        if self.IsOverPosition(targetJoint) == False:
            return

        self.recoveryTargetJointStartPos[targetJoint] = self.robot[targetJoint]["position"]
        self.recoveryTargetJoint.append(targetJoint)
        j = self.nameDict[targetJoint] + 1
        cmd = unicode("brake " + str(j) + " 0")     # brake off
        self.send_command(cmd)

    def StopRecoveryOverPositionSequence(self, targetJoint):
        self.recoveryTargetJoint.remove(targetJoint)
        j = self.nameDict[targetJoint] + 1
        cmd = unicode("brake " + str(j) + " 1")         # brake on
        self.send_command(cmd)

    def RecoveryOverPositionCallback(self):
        for k,v in self.nameDict.items():
            if self.IsOverPosition(k) == True:
                self.button_recoverOverPos[k].setEnabled(True)
            else:
                self.button_recoverOverPos[k].setEnabled(False)

        if self.recoveryTargetJoint == []:
            return
        for name in self.recoveryTargetJoint:
            print "Recovery: " + name + " sequence"
            j = self.nameDict[name] + 1
            diffPos = numpy.abs(self.recoveryTargetJointStartPos[name] - self.robot[name]["position"])
            print "             diffPos: ", diffPos
            if diffPos < numpy.radians(5.0):
                continue
            cmd = unicode("reset " + str(j))
            self.send_command(cmd)
            rospy.sleep(0.1)
            if self.IsOverPosition(name) == False:
                self.StopRecoveryOverPositionSequence(name)

    
    def jointType_to_str(self, value):
        if value == eJointType.JOINT_TYPE_ROT: return "Rot"
        elif value == eJointType.JOINT_TYPE_GRIPPER_SLIDE: return "GrpS"
        elif value == eJointType.JOINT_TYPE_GRIPPER_ROT: return "GrpR"
        return "Unknown"

    def comStatus_to_item(self, value):
        color = QColor(Qt.black)
        if value == eComStatus.STOP:
            s = "Stop"
        elif value == eComStatus.OK:
            s = "OK"
            color = QColor(65, 205, 82)    # light green
        elif value == eComStatus.TIMEOUT_ERROR:
            s = "Timeout"
            color = QColor(Qt.red)
        elif value == eComStatus.CRC_ERROR:
            s = "CRCErr"
            color = QColor(Qt.red)
        else:
            s = "UnknownErr"
            color = QColor(Qt.red)
        item = QTableWidgetItem(s)
        item.setForeground(color)
        return item

    def systemMode_to_item(self, value):
        color = QColor(Qt.black)
        if value == eSystemMode.STOP:
            s = "Stop"
        elif value == eSystemMode.READY:
            s = "Ready"
        elif value == eSystemMode.RUN:
            s = "Run"
            color = QColor(65, 205, 82)    # light green
        elif value == eSystemMode.ERROR:
            s = "Error"
            color = QColor(Qt.red)
        else:
            s = "Unknown"
        item = QTableWidgetItem(s)
        item.setForeground(color)
        return item

    def ctrlMode_to_str(self, value):
        if value == eHighCtrlMode.TRAJ: return "Traj"
        elif value == eHighCtrlMode.VELOCITY: return "Vel"
        elif value == eHighCtrlMode.CURRENT: return "Cur"
        elif value == eHighCtrlMode.EXTERNAL_FORCE_FOLLOWING: return "ExtFF"
        elif value == eHighCtrlMode.ONLINE_TRAJ: return "OnTraj"
        elif value == eHighCtrlMode.POSITION: return "Pos"
        return "Unknown"

    def trjStatus_to_str(self, value):
        if value == eTrajStatus.STOP: return "Stop"
        elif value == eTrajStatus.START: return "Start"
        elif value == eTrajStatus.RUNNING: return "Running"
        elif value == eTrajStatus.NEXT_POINT: return "NextPoint"
        elif value == eTrajStatus.COMPLETE: return "Complete"
        elif value == eTrajStatus.CANCEL_REQUEST: return "CancelReq"
        elif value == eTrajStatus.CANCEL_COMPLETE: return "Canceled"
        elif value == eTrajStatus.BUFFER_OVERFLOW_ERROR: return "BufOver"
        elif value == eTrajStatus.BUFFER_UNDERFLOW_ERROR: return "BufUnder"
        elif value == eTrajStatus.INVALID_POSITION_ERROR: return "InvPos"
        elif value == eTrajStatus.INVALID_VELOCITY_ERROR: return "InvVel"
        elif value == eTrajStatus.INVALID_ACCELERATION_ERROR: return "InvAcc"
        elif value == eTrajStatus.INVALID_TIME_ERROR: return "InvTime"
        elif value == eTrajStatus.INVALID_ITRP_MODE_ERROR: return "InvItrp"
        elif value == eTrajStatus.GENERATE_ERROR: return "GenErr"
        elif value == eTrajStatus.START_ERROR: return "StartErr"
        elif value == eTrajStatus.POSDIFF_DETECT_ERROR: return "PosDifDet"
        elif value == eTrajStatus.NOT_COMPLETE_ERROR: return "NotComp"
        elif value == eTrajStatus.VELDIFF_DETECT_ERROR: return "VelDifDet"
        elif value == eTrajStatus.SAFETY_STOP_FAILED_EM_ERROR: return "StopFailed"
        return "Unknown"

    def ewStatus_to_item(self, value):
        text = ""
        color = QColor(Qt.black)
        if value == 0:
            text = "NoError"
        else:
            color = QColor(Qt.red)
            if (value & (1 << eSlaveErrorWarningStatus.ERROR_OVER_POSITION)) != 0: text = text + "sOvPos, "
            if (value & (1 << eSlaveErrorWarningStatus.ERROR_OVER_VELOCITY)) != 0: text = text + "sOvVel, "
            if (value & (1 << eSlaveErrorWarningStatus.ERROR_OVER_CURRENT)) != 0: text = text + "sOvCur, "
            if (value & (1 << eSlaveErrorWarningStatus.ERROR_OVER_INTEGRATED_CURRENT)) != 0: text = text + "sOvIntCur, "
            if (value & (1 << eSlaveErrorWarningStatus.ERROR_OVER_EFFORT)) != 0: text = text + "sOvEft, "
            if (value & (1 << eSlaveErrorWarningStatus.ERROR_OVER_TEMPERATURE)) != 0: text = text + "sOvTmp, "
            if (value & (1 << eSlaveErrorWarningStatus.ERROR_OVER_VOLTAGE)) != 0: text = text + "sOvVol, "
            if (value & (1 << eSlaveErrorWarningStatus.ERROR_UNDER_VOLTAGE)) != 0: text = text + "sUnVol, "
            if (value & (1 << eSlaveErrorWarningStatus.ERROR_DRIVER_FAULT)) != 0: text = text + "sDrvFlt, "
            if (value & (1 << eSlaveErrorWarningStatus.ERROR_BRAKE_FAULT)) != 0: text = text + "sBrakeFlt, "
            if (value & (1 << eSlaveErrorWarningStatus.ERROR_MASTER_COM_LOST)) != 0: text = text + "sMstLost, "
            if (value & (1 << eSlaveErrorWarningStatus.ERROR_ENCD_IN_READ_FAIL)) != 0: text = text + "sEncInF, "
            if (value & (1 << eSlaveErrorWarningStatus.ERROR_ENCD_OUT_READ_FAIL)) != 0: text = text + "sEncOutF, "
            if (value & (1 << eSlaveErrorWarningStatus.ERROR_MANUAL_BRAKE_OFF)) != 0: text = text + "sManualBrakeOff, "
            if (value & (1 << eSlaveErrorWarningStatus.ERROR_FUSE_BLOWOUT)) != 0: text = text + "sFuseBlowout, "
            if (value & (1 << eSlaveErrorWarningStatus.WARNING_EFFORT_GET_FAIL)) != 0: text = text + "sEftGetF, "
            if (value & (1 << eMasterErrorWarningStatus.ERROR_OVER_POSITION)) != 0: text = text + "mOvPos, "
            if (value & (1 << eMasterErrorWarningStatus.ERROR_OVER_VELOCITY)) != 0: text = text + "mOvVel, "
            if (value & (1 << eMasterErrorWarningStatus.ERROR_OVER_CURRENT)) != 0: text = text + "mOvCur, "
            if (value & (1 << eMasterErrorWarningStatus.ERROR_OVER_EFFORT)) != 0: text = text + "mOvEft, "
            if (value & (1 << eMasterErrorWarningStatus.ERROR_OVER_TEMPERATURE)) != 0: text = text + "mOvTmp, "
            if (value & (1 << eMasterErrorWarningStatus.ERROR_OVER_POWER_CONSUMPTION)) != 0: text = text + "mOvPwr, "
            if (value & (1 << eMasterErrorWarningStatus.ERROR_EMERGENCY_STOP)) != 0: text = text + "mEmgStop, "
            if (value & (1 << eMasterErrorWarningStatus.ERROR_EXT_EMERGENCY_STOP)) != 0: text = text + "mExtEmgStop, "
            if (value & (1 << eMasterErrorWarningStatus.ERROR_HOST_COM_LOST)) != 0: text = text + "mHostLost, "
            if (value & (1 << eMasterErrorWarningStatus.ERROR_ETHERNET_INIT_FAILED)) != 0: text = text + "mEthInitF, "
            if (value & (1 << eMasterErrorWarningStatus.ERROR_OUT_OF_CONTROL)) != 0: text = text + "mOutCtrl, "
            if (value & (1 << eMasterErrorWarningStatus.ERROR_RESERVE2)) != 0: text = text + "mER2, "
            if (value & (1 << eMasterErrorWarningStatus.ERROR_RESERVE3)) != 0: text = text + "mER3, "
            if (value & (1 << eMasterErrorWarningStatus.ERROR_RESERVE4)) != 0: text = text + "mER4, "
            if (value & (1 << eMasterErrorWarningStatus.ERROR_RESERVE5)) != 0: text = text + "mER5, "
            if (value & (1 << eMasterErrorWarningStatus.ERROR_RESERVE6)) != 0: text = text + "mER6, "
            text = text[0:len(text)-2]

        item = QTableWidgetItem(text)
        item.setForeground(color)
        return item

class eJointType:
    JOINT_TYPE_ROT = 0
    JOINT_TYPE_GRIPPER_SLIDE = 1
    JOINT_TYPE_GRIPPER_ROT = 2

class eComStatus:
    STOP = 0          # 通信停止
    OK = 1            # 通信正常
    TIMEOUT_ERROR = 2 # 通信タイムアウトエラー
    CRC_ERROR = 3     # CRC不一致エラー
    UNKNOWN_ERROR = 4 # 不明エラー

class eSystemMode:
    STOP = 0
    READY = 1
    RUN = 2
    ERROR = 3

class eHighCtrlMode:
    TRAJ = 0 # 軌道制御
    VELOCITY = 1 # 速度制御
    CURRENT = 2 # 電流制御
    EXTERNAL_FORCE_FOLLOWING = 5 # 外力追従制御モード
    ONLINE_TRAJ = 7 # オンライン軌道制御
    ROTATION_FIXED = 10 # 手先姿勢固定制御
    POSITION = 20 # 位置制御

class eTrajStatus:
    STOP = 0
    START = 1
    RUNNING = 2
    NEXT_POINT = 3
    COMPLETE = 4
    CANCEL_REQUEST = 5
    CANCEL_COMPLETE = 6
    ERROR_INDEX_BEGIN = 100
    BUFFER_OVERFLOW_ERROR = 101
    BUFFER_UNDERFLOW_ERROR = 102
    INVALID_POSITION_ERROR = 103
    INVALID_VELOCITY_ERROR = 104
    INVALID_ACCELERATION_ERROR = 105
    INVALID_TIME_ERROR = 106
    INVALID_ITRP_MODE_ERROR = 107
    GENERATE_ERROR = 108
    START_ERROR = 109
    POSDIFF_DETECT_ERROR = 110
    NOT_COMPLETE_ERROR = 111
    VELDIFF_DETECT_ERROR = 112
    EMERGENCY_ERROR_INDEX_BEGIN = 200
    SAFETY_STOP_FAILED_EM_ERROR = 201


class eSlaveErrorWarningStatus:
    ERROR_OVER_POSITION = 0 # 過位置エラー
    ERROR_OVER_VELOCITY = 1 # 過速度エラー
    ERROR_OVER_CURRENT = 2 # 過電流エラー
    ERROR_OVER_INTEGRATED_CURRENT = 3 # 過積算電流エラー
    ERROR_OVER_EFFORT = 4 # 過トルクor過力エラー
    ERROR_OVER_TEMPERATURE = 5 # 過熱エラー
    ERROR_OVER_VOLTAGE = 6 # Over Voltage Error
    ERROR_UNDER_VOLTAGE = 7 # Under Voltage Error
    ERROR_DRIVER_FAULT = 8 # Driver Fault Error
    ERROR_BRAKE_FAULT = 9 # Brake Fault Error
    ERROR_MASTER_COM_LOST = 10 # with Master Communication Lost
    ERROR_ENCD_IN_READ_FAIL = 11 # 入力軸エンコーダ角読み取り失敗エラー
    ERROR_ENCD_OUT_READ_FAIL = 12 # 出力軸エンコーダ角読み取り失敗エラー
    ERROR_MANUAL_BRAKE_OFF = 13 # 手動ブレーキ解除状態エラー
    ERROR_FUSE_BLOWOUT = 14 # ヒューズ切れエラー
    WARNING_EFFORT_GET_FAIL = 15 # トルクor力センサ値取得失敗エラー

class eMasterErrorWarningStatus:
    ERROR_OVER_POSITION = 16 # 過位置エラー
    ERROR_OVER_VELOCITY = 17 # 過速度エラー
    ERROR_OVER_CURRENT = 18 # 過電流エラー
    ERROR_OVER_EFFORT = 19 # 過トルクor力エラー
    ERROR_OVER_TEMPERATURE = 20 # 過熱エラー
    ERROR_OVER_POWER_CONSUMPTION = 21 # Over power consumption
    ERROR_EMERGENCY_STOP = 22 # Emergency stop
    ERROR_EXT_EMERGENCY_STOP = 23 # External Emergency stop
    ERROR_HOST_COM_LOST = 24 # with Host Communication Lost
    ERROR_ETHERNET_INIT_FAILED = 25 # Ethernet initialize failed
    ERROR_OUT_OF_CONTROL = 26 # Out of control
    ERROR_RESERVE2 = 27 # Reserve
    ERROR_RESERVE3 = 28 # Reserve
    ERROR_RESERVE4 = 29 # Reserve
    ERROR_RESERVE5 = 30 # Reserve
    ERROR_RESERVE6 = 31 # Reserve
