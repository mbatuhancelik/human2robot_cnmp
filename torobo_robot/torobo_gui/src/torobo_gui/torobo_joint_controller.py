#!/usr/bin/env python
## -*- coding: utf-8 -*-


import os
import rospy
import rosparam
import rospkg
import numpy as np
import copy
from collections import OrderedDict

import tf
import tf2_ros
import moveit_commander
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from moveit_msgs.msg import RobotState, PositionIKRequest
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from moveit_msgs.msg import MoveItErrorCodes

from joint_limits_urdf import get_joint_limits

from torobo_motion_manager import teaching_point_manager
from torobo_motion_manager import home_position_manager

from torobo_common import srdf_parser

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *

from ast import parse
import yaml

from control_msgs.msg import GripperCommand, GripperCommandActionGoal
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from actionlib_msgs.msg import GoalStatus


class ToroboJointController(Plugin):

    def __init__(self, context):

        super(self.__class__, self).__init__(context)
        self.setObjectName('ToroboJointController')

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
        ui_file = os.path.join(rospkg.RosPack().get_path('torobo_gui'), 'resource', 'torobo_joint_controller.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ToroboJointControllerUi')

        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)
        self.serial_number = context.serial_number()

        # Create sub widgets
        self.create()

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

    def create(self):

        # Init values
        self._nameSpace = rospy.get_namespace()
        if (self._nameSpace == "/"):
            self._nameSpace = "torobo"
        if self._nameSpace[0] == "/":
            self._nameSpace = self._nameSpace[1:]
        if self._nameSpace[-1] == "/":
            self._nameSpace = self._nameSpace[:-1]
        self._controller_info = self.get_controller_info("move_group/controller_list")
        self._joint_group_double_editor_list = []
        self._trajectory_command_publisher = None
        self._display_planned_path_publisher = None
        self._joint_state = None
        self._current_tp = None

        # ros publisher
        self._trajectory_command_publisher = {}
        for controller in self._controller_info.keys():
            if "gripper" in controller:
                self._trajectory_command_publisher[controller] = actionlib.SimpleActionClient(controller + "/gripper_cmd", GripperCommandAction)
            else:
                self._trajectory_command_publisher[controller] = rospy.Publisher(controller + "/command", JointTrajectory, queue_size=1)
        self._display_planned_path_publisher = rospy.Publisher("move_group/display_planned_path", DisplayTrajectory, queue_size=1)
        # ros subscriber
        self._joint_state_subscriber = rospy.Subscriber("joint_state_server/joint_states", JointState, self.joint_state_callback, queue_size=1)

        # comboBox (ControllerNameSpace)
        self._widget.comboBoxControllerNamespace.addItem(self._nameSpace)

        # comboBox (ControllerName)
        for controller in self._controller_info.keys():
            self._widget.comboBoxControllerName.addItem(controller)
        self._widget.comboBoxControllerName.activated[str].connect(self.combobox_controllername_activated)
        if (self.serial_number <= self._widget.comboBoxControllerName.count()):
            self._widget.comboBoxControllerName.setCurrentIndex(-1)

        # comboBox (TpSelect)
        self._comboBoxTpName = EnhancedComboBox()
        self._comboBoxTpName.selected.connect(self.combobox_tpname_event)
        self._comboBoxTpName.focusReleased.connect(self.combobox_tpname_event)
        self._comboBoxTpName.editingFinished.connect(self.combobox_tpname_event)
        self._comboBoxTpName.setEnabled(False)
        self._comboBoxTpName.setEditable(False)
        self._widget.gridLayoutTP.addWidget(self._comboBoxTpName, 0, 0)

        # button (Save, Delete)
        self._widget.pushButtonTpRegister.clicked.connect(self.pushbutton_tp_clicked)
        self._widget.pushButtonTpRegister.setEnabled(False)
        self._widget.pushButtonTpDelete.clicked.connect(self.pushbutton_tp_clicked)
        self._widget.pushButtonTpDelete.setEnabled(False)

        # button (FileSave, FileOpen)
        self._widget.pushButtonTpFileSave.setIcon(QIcon.fromTheme('document-save-as'))
        self._widget.pushButtonTpFileSave.clicked.connect(self.pushbutton_tp_clicked)
        self._widget.pushButtonTpFileSave.setEnabled(False)
        self._widget.pushButtonTpFileOpen.setIcon(QIcon.fromTheme('document-open'))
        self._widget.pushButtonTpFileOpen.clicked.connect(self.pushbutton_tp_clicked)
        self._widget.pushButtonTpFileOpen.setEnabled(False)

        # button (Connect)
        self._widget.pushButtonConnectThis.toggled.connect(self.pushButtonConnect_toggled)
        self._widget.pushButtonConnectThis.setEnabled(False)
        self._widget.pushButtonConnectWhole.toggled.connect(self.pushButtonConnect_toggled)
        self._widget.pushButtonConnectWhole.setEnabled(False)

        # double_editors (joint_group)
        self._widget.joint_group.setLayout(QVBoxLayout())

        # comboBox (Preset Position)
        self._widget.comboBoxPresetPosition.activated[str].connect(self.combobox_preset_position_activated)

        # comboBox (AngleUnit)
        self._widget.comboBoxAngleUnit.activated[str].connect(self.combobox_angle_unit_activated)

        # comboBox (Reference/Target frame)
        self._widget.comboBoxReferenceFrame.activated[str].connect(self.combobox_frame_activated)
        self._widget.comboBoxTargetFrame.activated[str].connect(self.combobox_frame_activated)

        # comboBox (step of xyz[meter])
        self._widget.comboBoxXyzStep.clear()
        self._widget.comboBoxXyzStep.addItem("0.001")
        self._widget.comboBoxXyzStep.addItem("0.002")
        self._widget.comboBoxXyzStep.addItem("0.003")
        self._widget.comboBoxXyzStep.addItem("0.004")
        self._widget.comboBoxXyzStep.addItem("0.005")
        self._widget.comboBoxXyzStep.setCurrentIndex(0)

        # comboBox (step of rpy[degree])
        self._widget.comboBoxRpyStep.clear()
        self._widget.comboBoxRpyStep.addItem("1.0")
        self._widget.comboBoxRpyStep.addItem("2.0")
        self._widget.comboBoxRpyStep.addItem("3.0")
        self._widget.comboBoxRpyStep.setCurrentIndex(0)

        # pushButton for jog movement (dx, dy, dz, rx, ry, rz)
        self._widget.pushButtonXPlus.clicked.connect(self.pushbutton_jog_movement_clicked)
        self._widget.pushButtonXMinus.clicked.connect(self.pushbutton_jog_movement_clicked)
        self._widget.pushButtonYPlus.clicked.connect(self.pushbutton_jog_movement_clicked)
        self._widget.pushButtonYMinus.clicked.connect(self.pushbutton_jog_movement_clicked)
        self._widget.pushButtonZPlus.clicked.connect(self.pushbutton_jog_movement_clicked)
        self._widget.pushButtonZMinus.clicked.connect(self.pushbutton_jog_movement_clicked)
        self._widget.pushButtonRXPlus.clicked.connect(self.pushbutton_jog_movement_clicked)
        self._widget.pushButtonRXMinus.clicked.connect(self.pushbutton_jog_movement_clicked)
        self._widget.pushButtonRYPlus.clicked.connect(self.pushbutton_jog_movement_clicked)
        self._widget.pushButtonRYMinus.clicked.connect(self.pushbutton_jog_movement_clicked)
        self._widget.pushButtonRZPlus.clicked.connect(self.pushbutton_jog_movement_clicked)
        self._widget.pushButtonRZMinus.clicked.connect(self.pushbutton_jog_movement_clicked)

        # timer to update double_editor by callbackjoint_states) data
        self._timer = QTimer()
        self._timer.timeout.connect(self._on_timer)
        self._timer.start(50)

        # jog movement params
        self._limit_angle_of_jog_movement = np.radians(5.0)
        self._transition_time_of_jog_movement = 0.5

    def destroy(self):
        for key in self._trajectory_command_publisher.keys():
            if hasattr(self._trajectory_command_publisher[key], 'unregister'):
                self._trajectory_command_publisher[key].unregister()
                self._trajectory_command_publisher[key] = None
        if self._display_planned_path_publisher is not None:
            self._display_planned_path_publisher.unregister()
            self._display_planned_path_publisher = None
        if self._joint_state_subscriber is not None:
            self._joint_state_subscriber.unregister()

    # callback for ROS's subscriber
    def joint_state_callback(self, msg):
        self._joint_state = msg

    def get_joint_state(self):
        return self._joint_state

    # callback for Timer
    def _on_timer(self):
        if self._joint_group_double_editor_list is not None:
            for de in self._joint_group_double_editor_list:
                position = self._joint_state.position[self.get_joint_state().name.index(de.text())]
                de.updateCurrentValue.emit(position)

    def get_controller_info(self, rosparam_name):
        info = {}
        data = rospy.get_param(rosparam_name)
        for l in data:
            fullname = l["name"]
            #if( "gripper" in fullname):
            #    continue
            name = fullname.replace(self._nameSpace.replace("/", ""), "").lstrip("/")
            dic = {}
            id = 0
            for j in l["joints"]:
                dic[j] = id
                id += 1
            info[name] = sorted(dic.items(), key=lambda x: x[1])
        info = OrderedDict(sorted(info.items(), key=lambda x: x[0]))
        return info

    def is_valid_tp_name(self, name):
        import re
        if re.match(r"^\w+$", name) is None:
            return False
        return self.is_valid_rosparam_name(name)

    def is_valid_rosparam_name(self, name):
        c = name[0]
        if not (c.isalpha() or c == '/' or c == '~'):
            return False
        for c in name:
            if not (c.isalnum() or c == '/' or c == '~'):
                return False
        return True 

    def combobox_controllername_activated(self):
        sender = self.sender()
        controller_name = sender.currentText()

        if len(self.get_joint_state().name) == 0:
            rospy.loginfo("joint_state has not been ready")
            return

        joint_limits = get_joint_limits()

        pre_tp_name = ""
        if self._comboBoxTpName.currentIndex() > 0: # 0 means "<New>"
            pre_tp_name = self._comboBoxTpName.currentText()

        # clear DoubleEditors
        def clearLayout(layout):
            while layout.count():
                child = layout.takeAt(0)
                if child.widget() is not None:
                    child.widget().deleteLater()
                elif child.layout() is not None:
                    clearLayout(child.layout())
        clearLayout(self._widget.joint_group.layout())

        if "gripper" in controller_name:
            current_unit = self._widget.comboBoxAngleUnit.currentText()
            self._widget.comboBoxAngleUnit.clear()
            self._widget.comboBoxAngleUnit.addItem("meter")
            self._widget.comboBoxAngleUnit.setCurrentIndex(0)
            self._widget.comboBoxPresetPosition.clear()
            self._widget.comboBoxPresetPosition.addItem("Current")
            self._widget.comboBoxPresetPosition.addItem("Zero")
            self._widget.comboBoxPresetPosition.setCurrentIndex(0)
        else:
            current_unit = self._widget.comboBoxAngleUnit.currentText()
            self._widget.comboBoxAngleUnit.clear()
            self._widget.comboBoxAngleUnit.addItem("degree")
            self._widget.comboBoxAngleUnit.addItem("radian")
            index = self._widget.comboBoxAngleUnit.findText(current_unit)
            if index < 0:
                index = 0
            self._widget.comboBoxAngleUnit.setCurrentIndex(index)
            self._widget.comboBoxPresetPosition.clear()
            self._widget.comboBoxPresetPosition.addItem("Current")
            self._widget.comboBoxPresetPosition.addItem("Zero")
            self._widget.comboBoxPresetPosition.addItem("Home")
            self._widget.comboBoxPresetPosition.setCurrentIndex(0)

        # update comboBoxTpList
        self._comboBoxTpName.clear()
        if "gripper" in controller_name:
            pass
        else:
            self._comboBoxTpName.addItem("<New>")
            names = teaching_point_manager.GetTeachingPointNamesFromRosParam("/" + self._nameSpace + "/" + controller_name)
            if names is not None:
                for name in names:
                    self._comboBoxTpName.addItem(name)

        self._comboBoxTpName.setCurrentIndex(-1)
        tp = None
        if pre_tp_name == "<New>" or pre_tp_name == "":
            pass
        else:
            tp_index = self._comboBoxTpName.findText(pre_tp_name)
            if tp_index > 0:
                self._comboBoxTpName.setCurrentIndex(tp_index)
                tp = teaching_point_manager.GetTeachingPointFromRosParam("/" + self._nameSpace + "/" + controller_name, pre_tp_name)

        # add DoubleEditor
        self._joint_group_double_editor_list = []
        i = 0
        for key, value in self._controller_info[controller_name]:
            current_position = self.get_joint_state().position[self.get_joint_state().name.index(key)]
            if tp != None:
                current_position = tp.positions[i]
            i += 1
            de = DoubleEditor(key, joint_limits[key]["min_position"], joint_limits[key]["max_position"], current_position, current_position)
            de.targetValueChanged.connect(self.joint_group_double_editor_target_value_changed)
            self._widget.joint_group.layout().addWidget(de)
            self._joint_group_double_editor_list.append(de)

        # start move_group_commander service for forward kinematics
        move_group_name = str(controller_name[0:controller_name.find("_controller")])
        self._current_move_group_commander = moveit_commander.MoveGroupCommander(move_group_name, robot_description=rospy.get_namespace() +"robot_description", ns=rospy.get_namespace())
        rospy.wait_for_service('compute_fk')
        rospy.wait_for_service('compute_ik')
        try:
            self._moveit_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
            self._moveit_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s"%e)

        # switch enabled for other controls
        if len(self._joint_group_double_editor_list) > 0:
            self._widget.pushButtonConnectThis.setEnabled(True)
            if "gripper" in controller_name:
                self._widget.pushButtonConnectWhole.setEnabled(False)
            else:
                self._widget.pushButtonConnectWhole.setEnabled(True)
            self._comboBoxTpName.setEnabled(True)
            self._widget.pushButtonTpRegister.setEnabled(True)
            self._widget.pushButtonTpDelete.setEnabled(True)
            self._widget.pushButtonTpFileSave.setEnabled(True)
            self._widget.pushButtonTpFileOpen.setEnabled(True)
        else:
            self._widget.pushButtonConnectWhole.setEnabled(False)
            self._comboBoxTpName.setEnabled(False)
            self._widget.pushButtonTpRegister.setEnabled(False)
            self._widget.pushButtonTpDelete.setEnabled(False)
            self._widget.pushButtonTpFileSave.setEnabled(False)
            self._widget.pushButtonTpFileOpen.setEnabled(False)

        # update reference/target frame
        robot_commander = moveit_commander.RobotCommander(robot_description="robot_description" , ns=rospy.get_namespace())
        links = robot_commander.get_link_names()
        self._widget.comboBoxReferenceFrame.clear()
        self._widget.comboBoxTargetFrame.clear()
        for link in links:
            self._widget.comboBoxReferenceFrame.addItem(link)
            self._widget.comboBoxTargetFrame.addItem(link)
        self._widget.comboBoxReferenceFrame.setCurrentIndex(self._widget.comboBoxReferenceFrame.findText("world"))
        eef_link = self._current_move_group_commander.get_end_effector_link()
        self._widget.comboBoxTargetFrame.setCurrentIndex(self._widget.comboBoxTargetFrame.findText(eef_link))

        self.update_xyzrpy()
        self.publish()

    def combobox_tpname_event(self, signal_id):
        sender = self.sender()
        tp_name = sender.currentText()

        if signal_id == EnhancedComboBox.SIGNAL_SELECTED:
            self._current_tp = None
            if tp_name == "<New>":
                sender.setEditable(True)
                sender.lineEdit().selectAll()
                return
            else:
                sender.setEditable(False)
            rosparam_name = str(self._widget.comboBoxControllerName.currentText())
            if not self.is_valid_tp_name(tp_name):
                return
            point = teaching_point_manager.GetTeachingPointFromRosParam(rosparam_name, tp_name)
            print point
            if point != None:
                self._current_tp = point
                for i in range(len(self._joint_group_double_editor_list)):
                    self._joint_group_double_editor_list[i].setTargetValue(point.positions[i])
                    self._joint_group_double_editor_list[i]._on_slider_value_changed()
            self._widget.pushButtonTpRegister.setFont(QFontNormal())

        elif signal_id == EnhancedComboBox.SIGNAL_EDITING_FINISHED:
            if tp_name == "<New>":
                sender.setEditable(False)
                sender.setCurrentIndex(-1)
                self._current_tp = None
            elif not self.is_valid_tp_name(tp_name):
                sender.setEditable(False)
                sender.removeItem(self._comboBoxTpName.currentIndex())
                sender.setCurrentIndex(-1)
                self._current_tp = None
            else:
                point = JointTrajectoryPoint()
                for de in self._joint_group_double_editor_list:
                    point.positions.append(de.targetValue())
                rosparam_name = str(self._widget.comboBoxControllerName.currentText())
                teaching_point_manager.RecordTeachingPointToRosParam(rosparam_name, tp_name, point)
                self._comboBoxTpName.setEditable(False)
                self._current_tp = point
            self._widget.pushButtonTpRegister.setFont(QFontNormal())

        elif signal_id == EnhancedComboBox.SIGNAL_FOCUS_RELEASED:
            sender.setEditable(False)
            if tp_name == "<New>":
                sender.setCurrentIndex(-1)

        self.update_xyzrpy()
        self.publish()

    def pushbutton_tp_clicked(self):
        sender = self.sender()

        # Save button
        if sender == self._widget.pushButtonTpRegister:
            if self._comboBoxTpName.currentIndex() < 0:
                return
            if self._comboBoxTpName.currentText() == "<New>":
                return
            tp_name = str(self._comboBoxTpName.currentText())
            point = JointTrajectoryPoint()
            for de in self._joint_group_double_editor_list:
                point.positions.append(de.targetValue())
            rosparam_name = str(self._widget.comboBoxControllerName.currentText())
            teaching_point_manager.RecordTeachingPointToRosParam(rosparam_name, tp_name, point)
            sender.setFont(QFontNormal())

        # Delete button
        elif sender == self._widget.pushButtonTpDelete:
            if self._comboBoxTpName.currentIndex() < 0:
                return
            if self._comboBoxTpName.currentText() == "<New>":
                return
            tp_name = str(self._comboBoxTpName.currentText())
            rosparam_name = str(self._widget.comboBoxControllerName.currentText())
            teaching_point_manager.DeleteTeachingPointFromRosParam(rosparam_name, tp_name)
            self._comboBoxTpName.removeItem(self._comboBoxTpName.currentIndex())
            sender.setFont(QFontNormal())

        # FileSave button
        elif sender == self._widget.pushButtonTpFileSave:
            loadfile = QFileDialog.getSaveFileName(self._widget, "Save rosparam", "~/torobo_teaching_param.yaml", "Yaml (*.yaml)")
            fileName = loadfile[0]
            if fileName != "":
                param = {}
                for (controller_name, dic) in self._controller_info.items():
                    if ("gripper" in controller_name):
                        continue
                    param[controller_name] = {}
                    tps = rospy.get_param(controller_name + "/teaching_points", None)
                    trajs = rospy.get_param(controller_name + "/teaching_trajectories", None)
                    if (tps != None):
                        param[controller_name]['teaching_points'] = tps
                    if (trajs != None):
                        param[controller_name]['teaching_trajectories'] = trajs
                with open(fileName, "w") as outfile:
                    yaml.dump(param, outfile, default_flow_style=False)
                rospy.loginfo("Saved rosparam to [%s]" % fileName)
            else:
                rospy.loginfo("Not saved")

        # FileOpen button
        elif sender == self._widget.pushButtonTpFileOpen:
            loadfile = QFileDialog.getOpenFileName(self._widget, "Please select loading rosparam yaml file", "~/", "Yaml (*.yaml)")
            fileName = loadfile[0]
            if(os.path.exists(fileName) == False):
                rospy.loginfo("[%s] is invalid rosparam file." % fileName)
                return
            paramlist = rosparam.load_file(fileName)
            for params, ns in paramlist:
                rosparam.upload_params(ns, params)
            rospy.loginfo("Loaded rosparam from [%s]" % fileName)
            if self._widget.comboBoxControllerName.isEnabled():
                # update comboBoxTpList
                self._comboBoxTpName.clear()
                self._comboBoxTpName.addItem("<New>")
                tp_names = teaching_point_manager.GetTeachingPointNamesFromRosParam(str(self._widget.comboBoxControllerName.currentText()))
                if tp_names is not None:
                    for tp_name in tp_names:
                        self._comboBoxTpName.addItem(tp_name)
                self._comboBoxTpName.setCurrentIndex(-1)

                if len(self._joint_group_double_editor_list) > 0:
                    self._widget.pushButtonConnectThis.setEnabled(True)
                    self._widget.pushButtonConnectWhole.setEnabled(True)
                    self._comboBoxTpName.setEnabled(True)
                else:
                    self._widget.pushButtonConnectThis.setEnabled(False)
                    self._widget.pushButtonConnectWhole.setEnabled(False)
                    self._comboBoxTpName.setEnabled(False)

    def pushButtonConnect_toggled(self, checked):
        sender = self.sender()
        if sender == self._widget.pushButtonConnectThis:
            if checked:
                self._widget.comboBoxControllerNamespace.setEnabled(False)
                self._widget.comboBoxControllerName.setEnabled(False)
                self._comboBoxTpName.setEnabled(False)
                self._widget.comboBoxPresetPosition.setEnabled(False)
                self._widget.pushButtonTpRegister.setEnabled(False)
                self._widget.pushButtonTpDelete.setEnabled(False)
                self._widget.pushButtonTpFileSave.setEnabled(False)
                self._widget.pushButtonTpFileOpen.setEnabled(False)
                self._widget.pushButtonConnectWhole.setEnabled(False)
                for de in self._joint_group_double_editor_list:
                    de.setWorkingColor(True)
            else:
                self._widget.comboBoxControllerNamespace.setEnabled(True)
                self._widget.comboBoxControllerName.setEnabled(True)
                self._comboBoxTpName.setEnabled(True)
                self._widget.comboBoxPresetPosition.setEnabled(True)
                self._widget.pushButtonTpRegister.setEnabled(True)
                self._widget.pushButtonTpDelete.setEnabled(True)
                self._widget.pushButtonTpFileSave.setEnabled(True)
                self._widget.pushButtonTpFileOpen.setEnabled(True)
                controller_name = str(self._widget.comboBoxControllerName.currentText())
                if "gripper" in controller_name:
                    self._widget.pushButtonConnectWhole.setEnabled(False)
                else:
                    self._widget.pushButtonConnectWhole.setEnabled(True)
                for de in self._joint_group_double_editor_list:
                    de.setWorkingColor(False)

        elif sender == self._widget.pushButtonConnectWhole:
            if checked:
                self._widget.comboBoxControllerNamespace.setEnabled(False)
                self._widget.comboBoxControllerName.setEnabled(False)
                self._comboBoxTpName.setEnabled(False)
                self._widget.comboBoxPresetPosition.setEnabled(False)
                self._widget.pushButtonTpRegister.setEnabled(False)
                self._widget.pushButtonTpDelete.setEnabled(False)
                self._widget.pushButtonTpFileSave.setEnabled(False)
                self._widget.pushButtonTpFileOpen.setEnabled(False)
                self._widget.pushButtonConnectThis.setEnabled(False)
                for de in self._joint_group_double_editor_list:
                    de.setWorkingColor(True)
            else:
                self._widget.comboBoxControllerNamespace.setEnabled(True)
                self._widget.comboBoxControllerName.setEnabled(True)
                self._comboBoxTpName.setEnabled(True)
                self._widget.comboBoxPresetPosition.setEnabled(True)
                self._widget.pushButtonTpRegister.setEnabled(True)
                self._widget.pushButtonTpDelete.setEnabled(True)
                self._widget.pushButtonTpFileSave.setEnabled(True)
                self._widget.pushButtonTpFileOpen.setEnabled(True)
                self._widget.pushButtonConnectThis.setEnabled(True)
                for de in self._joint_group_double_editor_list:
                    de.setWorkingColor(False)

        self.publish()

    def combobox_preset_position_activated(self):
        sender = self.sender()
        text = str(sender.currentText())
        if text == "Zero":
            for de in self._joint_group_double_editor_list:
                de.setTargetValue(0.0)
                de._on_slider_value_changed()
        elif text == "Home":
            controller_name = str(self._widget.comboBoxControllerName.currentText())
            home_position, home_position_joint_name = home_position_manager.GetHomePosition(controller_name)
            for de in self._joint_group_double_editor_list:
                for i in range(len(home_position_joint_name)):
                    if(de.text() != home_position_joint_name[i]):
                        continue
                    de.setTargetValue(home_position.positions[i])
                    de._on_slider_value_changed()
                    break
        elif text == "Current":
            for de in self._joint_group_double_editor_list:
                de.setTargetValue(de.currentValue())
                de._on_slider_value_changed()

    def combobox_angle_unit_activated(self):
        sender = self.sender()
        self.update_xyzrpy()

    def combobox_frame_activated(self):
        self.update_xyzrpy()

    def pushbutton_jog_movement_clicked(self):
        sender = self.sender()
        sender.blockSignals(True)

        # get jog movement's transition and rotation
        xyz_step = float(self._widget.comboBoxXyzStep.currentText())
        rpy_step = float(self._widget.comboBoxRpyStep.currentText()) * np.pi / 180.0
        tx, ty, tz, rx, ry, rz = [0, 0, 0, 0, 0, 0]
        if sender == self._widget.pushButtonXPlus:
            tx = xyz_step
        elif sender == self._widget.pushButtonXMinus:
            tx = -xyz_step
        elif sender == self._widget.pushButtonYPlus:
            ty = xyz_step
        elif sender == self._widget.pushButtonYMinus:
            ty = -xyz_step
        elif sender == self._widget.pushButtonZPlus:
            tz = xyz_step
        elif sender == self._widget.pushButtonZMinus:
            tz = -xyz_step
        elif sender == self._widget.pushButtonRXPlus:
            rx = rpy_step
        elif sender == self._widget.pushButtonRXMinus:
            rx = -rpy_step
        elif sender == self._widget.pushButtonRYPlus:
            ry = rpy_step
        elif sender == self._widget.pushButtonRYMinus:
            ry = -rpy_step
        elif sender == self._widget.pushButtonRZPlus:
            rz = rpy_step
        elif sender == self._widget.pushButtonRZMinus:
            rz = -rpy_step

        # get current target pose
        robot_state = RobotState()
        robot_state.joint_state.name = []
        robot_state.joint_state.position = []
        for de in self._joint_group_double_editor_list:
            robot_state.joint_state.name.append(de.text())
            robot_state.joint_state.position.append(de.targetValue())
        fk_result = self._moveit_fk(Header(0, rospy.Time.now(), self._widget.comboBoxReferenceFrame.currentText()),
            [self._widget.comboBoxTargetFrame.currentText()], robot_state)
        pose = fk_result.pose_stamped[0].pose

        # update target pose
        pose.position.x += tx
        pose.position.y += ty
        pose.position.z += tz
        dq = tf.transformations.quaternion_from_euler(rx, ry, rz)
        q = tf.transformations.quaternion_multiply(dq, (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))  # deal with dq as root rotation
        pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        # create request
        req = PositionIKRequest()
        req.group_name = self._current_move_group_commander.get_name()
        req.robot_state = robot_state
        req.avoid_collisions = True
        req.ik_link_name = self._widget.comboBoxTargetFrame.currentText()
        req.pose_stamped.header.frame_id = self._widget.comboBoxReferenceFrame.currentText()
        req.pose_stamped.pose = pose
        req.timeout = rospy.Duration(0.2)

        # inquire result
        res = self._moveit_ik(req)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            rospy.logwarn("ToroboJointController : Jog movement solution is not found.")
        else:
            # check amount of movement
            check_amount_of_movement = True
            for de in self._joint_group_double_editor_list:
                if self._widget.pushButtonConnectThis.isChecked() or self._widget.pushButtonConnectWhole.isChecked():
                    # when servo is on, compare target with current robot state
                    cur_value = de.currentValue()
                else:
                    # when servo is on, compare target with current target
                    cur_value = de.targetValue()
                index = res.solution.joint_state.name.index(de.text())
                new_value = res.solution.joint_state.position[index]
                if abs(cur_value - new_value) > self._limit_angle_of_jog_movement:
                    check_amount_of_movement = False
                    break
            if check_amount_of_movement:
                # publish new target
                for de in self._joint_group_double_editor_list:
                    index = res.solution.joint_state.name.index(de.text())
                    de.setTargetValue(res.solution.joint_state.position[index])
                self.publish(self._transition_time_of_jog_movement)
            else:
                rospy.logwarn("ToroboJointController : Jog movement distance exceeds the limit.")

        sender.blockSignals(False)
        self.update_xyzrpy()

    def joint_group_double_editor_target_value_changed(self):
        if self._current_tp is not None:
            for i, de in enumerate(self._joint_group_double_editor_list):
                if self._current_tp.positions[i] != de.targetValue():
                    self._widget.pushButtonTpRegister.setFont(QFontBold())
                    break
        self.update_xyzrpy()
        self.publish()

    def update_xyzrpy(self):

        angle_unit = self._widget.comboBoxAngleUnit.currentText()

        # Update joint_group's value
        for de in self._joint_group_double_editor_list:
            de.setUnit(angle_unit)

        # Update xyzrpy value
        controller = str(self._widget.comboBoxControllerName.currentText())
        if "gripper" in controller:
            self._widget.doubleSpinBoxRoll.setDecimals(3)
            self._widget.doubleSpinBoxPitch.setDecimals(3)
            self._widget.doubleSpinBoxYaw.setDecimals(3)
            self._widget.doubleSpinBoxX.setValue(0.0)
            self._widget.doubleSpinBoxY.setValue(0.0)
            self._widget.doubleSpinBoxZ.setValue(0.0)
            self._widget.doubleSpinBoxRoll.setValue(0.0)
            self._widget.doubleSpinBoxPitch.setValue(0.0)
            self._widget.doubleSpinBoxYaw.setValue(0.0)
        else:
            if self._widget.comboBoxAngleUnit.currentText() == "radian":
                self._widget.doubleSpinBoxRoll.setDecimals(3)
                self._widget.doubleSpinBoxPitch.setDecimals(3)
                self._widget.doubleSpinBoxYaw.setDecimals(3)
            else:
                self._widget.doubleSpinBoxRoll.setDecimals(1)
                self._widget.doubleSpinBoxPitch.setDecimals(1)
                self._widget.doubleSpinBoxYaw.setDecimals(1)

            # calculate by moveit
            state = RobotState()
            state.joint_state.name = []
            state.joint_state.position = []
            for de in self._joint_group_double_editor_list:
                state.joint_state.name.append(de.text())
                state.joint_state.position.append(de.targetValue())
            fk_result = self._moveit_fk(Header(0, rospy.Time.now(), self._widget.comboBoxReferenceFrame.currentText()),
                [self._widget.comboBoxTargetFrame.currentText()], state)
            pose = fk_result.pose_stamped[0].pose
            euler = tf.transformations.euler_from_quaternion(
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            )
            self._widget.doubleSpinBoxX.setValue(pose.position.x)
            self._widget.doubleSpinBoxY.setValue(pose.position.y)
            self._widget.doubleSpinBoxZ.setValue(pose.position.z)
            if self._widget.comboBoxAngleUnit.currentText() == "radian":
                self._widget.doubleSpinBoxRoll.setValue(euler[0])
                self._widget.doubleSpinBoxPitch.setValue(euler[1])
                self._widget.doubleSpinBoxYaw.setValue(euler[2])
            else:
                self._widget.doubleSpinBoxRoll.setValue(np.degrees(euler[0]))
                self._widget.doubleSpinBoxPitch.setValue(np.degrees(euler[1]))
                self._widget.doubleSpinBoxYaw.setValue(np.degrees(euler[2]))

    def publish(self, transition_time=-1):

        def get_current_teaching_point_name():
            if self._comboBoxTpName.currentIndex() < 0:
                tp_name = ""
            tp_name = self._comboBoxTpName.currentText()
            if tp_name == "" or tp_name == "<New>":
                tp_name = None
            return tp_name

        def create_joint_trajectory_of_current_view(controller, transition_time):
            if "gripper" in controller:
                return None
            trajectory = JointTrajectory()
            trajectory.joint_names = []
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(transition_time)
            point.positions = []
            for de in self._joint_group_double_editor_list:
                trajectory.joint_names.append(de.text())
                point.positions.append(de.targetValue())
                point.velocities.append(0.0)
                point.accelerations.append(0.0)
                point.effort.append(0.0)
            trajectory.points.append(point)
            return trajectory

        def create_gripper_command_goal_of_current_view(controller):
            if "gripper" not in controller:
                return None
            goal = GripperCommandGoal()
            goal.command.position = self._joint_group_double_editor_list[0].targetValue()
            goal.command.max_effort = 30.0
            return goal

        def create_joint_trajectory_of_teaching_point(controller, tp, transition_time):
            if "gripper" in controller:
                return None
            trajectory = JointTrajectory()
            trajectory.joint_names = []
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(transition_time)
            point.positions = []
            tp_names = teaching_point_manager.GetTeachingPointNamesFromRosParam(controller)
            if tp in tp_names:
                tp_data = teaching_point_manager.GetTeachingPointFromRosParam(controller, tp)
                if tp_data is None:
                    return None
                for i, (key, value) in enumerate(self._controller_info[controller]):
                    trajectory.joint_names.append(key)
                    point.positions.append(tp_data.positions[i])
                    point.velocities.append(0.0)
                    point.accelerations.append(0.0)
                    point.effort.append(0.0)
            trajectory.points.append(point)
            return trajectory

        def extend_joint_trajectory(traj1, traj2):
            traj1.joint_names.extend(traj2.joint_names)
            traj1.points[0].positions.extend(traj2.points[0].positions)
            traj1.points[0].velocities.extend(traj2.points[0].velocities)
            traj1.points[0].accelerations.extend(traj2.points[0].accelerations)
            traj1.points[0].effort.extend(traj2.points[0].effort)
            return traj1

        if len(self._joint_group_double_editor_list) == 0:
            return

        if transition_time < 0:
            transition_time = self._widget.doubleSpinBoxTransitionTime.value()
        current_controller_name = str(self._widget.comboBoxControllerName.currentText())

        # create trajectory for Display Planned Path publisher
        if "gripper" not in current_controller_name:
            trajectory = create_joint_trajectory_of_current_view(current_controller_name, transition_time)
            tp_name = get_current_teaching_point_name()
            if tp_name is not None:
                for controller_name in self._controller_info.keys():
                    if controller_name == str(self._widget.comboBoxControllerName.currentText()):
                        continue
                    trj = create_joint_trajectory_of_teaching_point(controller_name, tp_name, transition_time)
                    if trj is not None:
                        trajectory = extend_joint_trajectory(trajectory, trj)
            rt = RobotTrajectory()
            rt.joint_trajectory = trajectory
            msg = DisplayTrajectory()
            msg.model_id = 'torobo'
            msg.trajectory.append(rt)
            msg.trajectory_start.joint_state = copy.deepcopy(self.get_joint_state()) # current value
            if self._display_planned_path_publisher is not None:
                self._display_planned_path_publisher.publish(msg)

        # create trajectory for single controller's trajectory command
        if self._widget.pushButtonConnectThis.isChecked():
            if "gripper" not in current_controller_name:
                trajectory = create_joint_trajectory_of_current_view(current_controller_name, transition_time)
                if trajectory is not None:
                    if self._trajectory_command_publisher[current_controller_name] is not None:
                        self._trajectory_command_publisher[current_controller_name].publish(trajectory)
            else:
                goal = create_gripper_command_goal_of_current_view(current_controller_name)
                if goal is not None:
                    if self._trajectory_command_publisher[current_controller_name] is not None:
                        self._trajectory_command_publisher[current_controller_name].send_goal(goal)
                        self._trajectory_command_publisher[current_controller_name].wait_for_result(timeout=rospy.Duration(1.0))

        # create trajectory for whole controllers' trajectory command
        if self._widget.pushButtonConnectWhole.isChecked():
            if "gripper" not in current_controller_name:
                tp_name = get_current_teaching_point_name()
                if tp_name is not None:
                    for controller_name in self._controller_info.keys():
                        if controller_name == str(self._widget.comboBoxControllerName.currentText()):
                            continue
                        trajectory = create_joint_trajectory_of_teaching_point(controller_name, tp_name, transition_time)
                        if trajectory is not None:
                            if self._trajectory_command_publisher[controller_name] is not None:
                                self._trajectory_command_publisher[controller_name].publish(trajectory)
                # publish other controllers' trajectory command
                trajectory = create_joint_trajectory_of_current_view(current_controller_name, transition_time)
                if trajectory is not None:
                    if self._trajectory_command_publisher[current_controller_name] is not None:
                        self._trajectory_command_publisher[current_controller_name].publish(trajectory)


# QFont to specify bold
class QFontBold(QFont):
    def __init__(self):
        super(QFontBold, self).__init__()
        self.setBold(True)
class QFontNormal(QFont):
    def __init__(self):
        super(QFontNormal, self).__init__()
        self.setBold(False)

# DoubleEditor (QFloatSlider/QFloatSpinBox with QFloatProgressBar)
class DoubleEditor(QWidget):
    
    targetValueChanged = Signal(float)
    updateCurrentValue = Signal(float)

    # each '*value' argument's unit must be "radian"
    def __init__(self, text, min_value, max_value, target_value, current_value):
        super(DoubleEditor, self).__init__()
        assert min_value < max_value, "DoubleEditor's minumum cannot be higher than maximum."

        self._text = text                   # Joint's name
        self._min_value = min_value         # Minimum value
        self._max_value = max_value         # Maximum value
        self._target_value = target_value   # Target value for slider
        self._current_value = current_value # Current value for progressbar
        self._unit = "radian"               # Unit for preview

        # Set gridLayout to use multiple overlay
        self.setLayout(QGridLayout())

        # label for joint name
        label_joint_name = QLabel(text)
        label_joint_name.setFont(QFontBold())
        self.layout().addWidget(label_joint_name, 0, 0)

        # progressbar for current value
        self.progressbar = QFloatProgressBar()
        self.progressbar.setRange(self._min_value, self._max_value)
        self.progressbar.setValue(self._current_value)
        self.layout().addWidget(self.progressbar, 0, 1)
        self.setWorkingColor(False)

        # label for minimum value
        self.label_min = QLabel(" {:.3f}".format(self._min_value))
        self.label_min.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.layout().addWidget(self.label_min, 0, 1)

        # label for maximum value
        self.label_max = QLabel("{:.3f} ".format(self._max_value))
        self.label_max.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.layout().addWidget(self.label_max, 0, 1)

        # slider for target value
        self.slider = QFloatSlider(Qt.Horizontal)
        self.slider.setRange(self._min_value, self._max_value)
        self.slider.setValue(self._target_value)
        self.layout().addWidget(self.slider, 0, 1)

        self.slider.setStyleSheet("""
        QSlider::groove:horizontal{
            border: 0px solid lightgrey;
            height: 0px;
            background: orange;
            margin: 0px 0;
            border-radius: 5px;
        }
        QSlider::handle:horizontal {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #b4b4b4, stop:1 #8f8f8f);
            border: 2px solid #5c5c5c;
            width: 6px;
            margin: -15px 0;
            border-radius: 5px;
        }
        QSlider::add-page:horizontal{
            border: 0px solid lightgrey;
            height: 0px;
            background: white;
            margin: 0px 0;
            border-radius: 5px;
        }
        """)

        # spinbox for target value
        self.spinbox = QFloatSpinBox()
        self.spinbox.setRange(self._min_value, self._max_value)
        self.spinbox.setValue(self._target_value)
        self.spinbox.setSingleStep(0.01)
        self.spinbox.setDecimals(3)
        self.spinbox.setFixedWidth(70)
        self.spinbox.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.layout().addWidget(self.spinbox, 0, 2)

        # connect between slider and spinbox
        self.slider.valueChanged.connect(self._on_slider_value_changed)
        self.spinbox.editingFinished.connect(self._on_spinbox_editing_finished)

        # Register ROS's subscriber
        self.updateCurrentValue.connect(self.setCurrentValue)
        #rospy.Subscriber("/joint_states", JointState, self.callback, queue_size=1)

    def setWorkingColor(self, flag):
        if flag:
            self.progressbar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #5c5c5c;
                border-radius: 5px;
                text-align: center;
                height: 2px;
                margin: 2px 0;
                background: white;
            }
            QProgressBar::chunk {
                /*background-color: #92D050;*/
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #92D050, stop:1 #70A030);
                width: 1px;
            }
            """)
        else:
            self.progressbar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #5c5c5c;
                border-radius: 5px;
                text-align: center;
                height: 2px;
                margin: 2px 0;
                background: white;
            }
            QProgressBar::chunk {
                /*background-color: #FFC000;*/
                /*background-color: #8f8f8f;*/
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #FFC000, stop:1 #CA9000);
                width: 1px;
            }
            """)

    def _on_slider_value_changed(self):
        value = self.slider.value()
        # Set radian's value
        if self._unit == "radian":
            self._target_value = value
        else:
            self._target_value = value * np.pi / 180.0
        # Update spinbox
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(value)
        self.spinbox.blockSignals(False)
        # Emit signal
        self.targetValueChanged.emit(value)

    def _on_spinbox_editing_finished(self):
        value = self.spinbox.value()
        # Set radian's value
        if self._unit == "radian":
            self._target_value = value
        else:
            self._target_value = value * np.pi / 180.0
        # Update slider
        self.slider.blockSignals(True)
        self.slider.setValue(value)
        self.slider.blockSignals(False)
        # Emit signal
        self.targetValueChanged.emit(value)

    # return joint name
    def text(self):
        return self._text

    # set target value. (unit must be "radian")        
    def setTargetValue(self, value):
        self._target_value = value
        self.spinbox.blockSignals(True)
        self.slider.blockSignals(True)
        if self._unit == "radian":
            self.spinbox.setValue(self._target_value)
            self.slider.setValue(self._target_value)
        else:
            self.spinbox.setValue(self._target_value * 180.0 / np.pi)
            self.slider.setValue(self._target_value * 180.0 / np.pi)
        self.spinbox.blockSignals(False)
        self.slider.blockSignals(False)

    # set current value. (unit must be "radian")        
    def setCurrentValue(self, value):
        self._current_value = value
        if self._unit == "radian":
            self.progressbar.setValue(self._current_value)
        else:
            self.progressbar.setValue(self._current_value * 180.0 / np.pi)

    # return target value. (unit is "radian")
    def targetValue(self):
        return self._target_value

    # return current value. (unit is "radian")
    def currentValue(self):
        return self._current_value

    # set unit (augument must be "radian" or "degree".)
    def setUnit(self, unit):
        self.slider.valueChanged.disconnect()
        self.spinbox.editingFinished.disconnect()

        self._unit = unit
        if self._unit == "radian":
            self.label_min.setText(" {:.3f}".format(self._min_value))
            self.label_max.setText("{:.3f} ".format(self._max_value))
            self.slider.setRange(self._min_value, self._max_value)
            self.spinbox.setRange(self._min_value, self._max_value)
            self.progressbar.setRange(self._min_value, self._max_value)
            self.progressbar.setDecimals(3)
            self.spinbox.setSingleStep(0.01)
            self.spinbox.setDecimals(3)
        elif self._unit == "degree":
            self.label_min.setText(" {:.1f}".format(self._min_value * 180.0 / np.pi))
            self.label_max.setText("{:.1f} ".format(self._max_value * 180.0 / np.pi))
            self.slider.setRange(self._min_value * 180 / np.pi, self._max_value * 180 / np.pi)
            self.spinbox.setRange(self._min_value * 180 / np.pi, self._max_value * 180 / np.pi)
            self.progressbar.setRange(self._min_value * 180 / np.pi, self._max_value * 180 / np.pi)
            self.progressbar.setDecimals(1)
            self.spinbox.setSingleStep(1)
            self.spinbox.setDecimals(1)
        else:
            self._unit = "radian" # meter
            self.label_min.setText(" {:.3f}".format(self._min_value))
            self.label_max.setText("{:.3f} ".format(self._max_value))
            self.slider.setRange(self._min_value, self._max_value)
            self.spinbox.setRange(self._min_value, self._max_value)
            self.progressbar.setRange(self._min_value, self._max_value)
            self.progressbar.setDecimals(3)
            self.spinbox.setSingleStep(0.01)
            self.spinbox.setDecimals(3)
        self.setTargetValue(self._target_value)
        self.setCurrentValue(self._current_value)

        self.slider.valueChanged.connect(self._on_slider_value_changed)
        self.spinbox.editingFinished.connect(self._on_spinbox_editing_finished)

    def unit(self):
        return self._unit


# Slider to deal with float value
class QFloatSlider(QSlider):

    def __init__(self, orientation):
        super(QFloatSlider, self).__init__(orientation)
        self._max_int = 10 ** 5
        super(QFloatSlider, self).setMinimum(0)
        super(QFloatSlider, self).setMaximum(self._max_int)
        self._min_value = -1000.0
        self._max_value = 1000.0

    @property
    def _value_range(self):
        return self._max_value - self._min_value

    def value(self):
        return float(super(QFloatSlider, self).value()) / self._max_int * self._value_range + self._min_value

    def setValue(self, value):
        super(QFloatSlider, self).setValue(int((value - self._min_value) / self._value_range * self._max_int))

    def setMinimum(self, value):
        self._min_value = value
        self.setValue(self.value())

    def setMaximum(self, value):
        self._max_value = value
        self.setValue(self.value())

    def setRange(self, min_value, max_value):
        assert min_value < max_value, "Slider's minimum cannot be higher than maximum."
        self.setMinimum(min_value)
        self.setMaximum(max_value)

    def minimum(self):
        return self._min_value

    def maximum(self):
        return self._max_value


# ProgressBar to deal with float value
class QFloatProgressBar(QProgressBar):

    def __init__(self):
        super(QFloatProgressBar, self).__init__()
        self._max_int = 10 ** 5
        super(QFloatProgressBar, self).setMinimum(0)
        super(QFloatProgressBar, self).setMaximum(self._max_int)
        self._min_value = 0.0
        self._max_value = 100.0
        self._format = '%.03f'
        self._float_value = 0.0
        self.valueChanged.connect(self.onValueChanged)

    def onValueChanged(self, value):
        self.setFormat(self._format % (self._float_value))

    @property
    def _value_range(self):
        return self._max_value - self._min_value

    def value(self):
        return float(super(QFloatProgressBar, self).value()) / self._max_int * self._value_range + self._min_value

    def setValue(self, value):
        self._float_value = value
        super(QFloatProgressBar, self).setValue(int(self._max_int * (value - self._min_value) / self._value_range))

    def setMinimum(self, value):
        self._min_value = value
        self.setValue(self.value())

    def setMaximum(self, value):
        self._max_value = value
        self.setValue(self.value())

    def setRange(self, min_value, max_value):
        assert min_value < max_value, "Progressbar's minimum cannot be higher than maximum."
        self.setMinimum(min_value)
        self.setMaximum(max_value)

    def minimum(self):
        return self._min_value

    def maximum(self):
        return self._max_value

    def setDecimals(self, decimal):
        self._format = '%.0' + str(decimal) + 'f'
        self.setFormat(self._format % (self._float_value))


# SpinBox to map mouse-left-click to signal of "editingFinished".
class QFloatSpinBox(QDoubleSpinBox):

    def __init__(self):
        super(QFloatSpinBox, self).__init__()
        self.mouse_left_pressed = False
        self.valueChanged.connect(self._on_value_changed)

    def mousePressEvent(self, e):
        super(QFloatSpinBox, self).mousePressEvent(e)
        if e.buttons() == Qt.LeftButton:
            self.mouse_left_pressed = True
            self.editingFinished.emit()
        else:
            self.mouse_left_pressed = False

    def mouseReleaseEvent(self, e):
        super(QFloatSpinBox, self).mouseReleaseEvent(e)
        self.mouse_left_pressed = False

    def _on_value_changed(self):
        if self.mouse_left_pressed:
            self.editingFinished.emit()


class EnhancedComboBox(QComboBox):
    """Enhanced version of QtGui.QComboBox which has an editingFinished-signal like QLineEdit."""
    selected = pyqtSignal(int)
    editingFinished = pyqtSignal(int)
    focusReleased = pyqtSignal(int)
    _popup = None # The lineedit's contextmenu while it is shown

    SIGNAL_SELECTED = 1
    SIGNAL_EDITING_FINISHED = 2
    SIGNAL_FOCUS_RELEASED = 3

    def __init__(self,parent=None):
        super(self.__class__, self).__init__(parent)
        self.setEditable(True)
        self.lineEdit().installEventFilter(self)
        self.activated[str].connect(self.activatedEvent)

    def activatedEvent(self):
        self.selected.emit(self.SIGNAL_SELECTED)

    def keyPressEvent(self,keyEvent):
        super(self.__class__, self).keyPressEvent(keyEvent)
        if keyEvent.key() == Qt.Key_Return or keyEvent.key() == Qt.Key_Enter:
            if not self.view().isVisible() and self._popup is None:
                self.editingFinished.emit(self.SIGNAL_EDITING_FINISHED)
  
    def focusOutEvent(self,focusEvent):
        if not self.view().isVisible() and self._popup is None:
            self.focusReleased.emit(self.SIGNAL_FOCUS_RELEASED)
        QComboBox.focusOutEvent(self,focusEvent)
  
    def eventFilter(self,object,event):
        if event.type() == QEvent.ContextMenu and object == self.lineEdit():
            self._popup = self.lineEdit().createStandardContextMenu()
            self._popup.exec_(event.globalPos())
            self._popup = None
            return True
        return False # don't stop the event
