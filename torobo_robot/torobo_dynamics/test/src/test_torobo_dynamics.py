#!/usr/bin/env python
# -*- coding: utf-8 -*-

PKG = 'torobo_dynamics'
NAME = 'test_torobo_dynamics'

import sys
import time
import unittest
import numpy
import rospy
from torobo_test.test_common import *

from sensor_msgs.msg import JointState
from torobo_msgs.msg import ToroboDynamics, ToroboJointState
from torobo_msgs.srv import SetPayloadParam

class TestToroboDynamics(unittest.TestCase):
    def __init__(self, *args):
        super(TestToroboDynamics, self).__init__(*args)
        rospy.init_node(NAME, anonymous=True)
        self._ns = rospy.get_namespace()
        self._create()
        self._wait_service_ready()
        self._wait_first_receive()
        print("Ready.")

    def setUp(self):
        self._set_timeout(3.0)
        for controller in self._controller_list:
            self._set_joint_positions(self._torobo_joint_state[controller].name, [0.0] * len(self._torobo_joint_state[controller].name), False)
        self._publish_joint_states()

    def tearDown(self):
        pass

    def _create(self):
        self._controller_list = []
        self._first_recv_dynamics = {}
        self._first_recv_joint_state = {}
        self._torobo_dynamics = {}
        self._torobo_dynamics_sub = {}
        self._torobo_joint_state = {}
        self._torobo_joint_state_sub = {}
        self._joint_states = JointState()
        controller_list = rospy.get_param(self._ns + "controller_list")
        for controller in controller_list:
            if("gripper" in controller):
                continue
            joint_names = rospy.get_param(self._ns + controller + "/joints", None)
            if(joint_names == None):
                continue
            self._controller_list.append(controller)
            self._joint_states.name.extend(joint_names)
            self._first_recv_dynamics[controller] = False
            self._first_recv_joint_state[controller] = False
            self._torobo_dynamics[controller] = ToroboDynamics()
            self._torobo_dynamics_sub[controller] = rospy.Subscriber(self._ns + controller + "/torobo_dynamics", ToroboDynamics, self._torobo_dynamics_callback, controller, queue_size=1)
            self._torobo_joint_state[controller] = ToroboJointState()
            self._torobo_joint_state_sub[controller] = rospy.Subscriber(self._ns + controller + "/torobo_joint_state", ToroboJointState, self._torobo_joint_state_callback, controller, queue_size=1)
        self._joint_states.position = [0.0] * len(self._joint_states.name)
        self._joint_states.velocity = [0.0] * len(self._joint_states.name)
        self._joint_states.effort = [0.0] * len(self._joint_states.name)
        self._joint_states_pub = rospy.Publisher(self._ns + "joint_states", JointState, queue_size=1)

    def _set_timeout(self, timeout_sec=3.0):
        self.timeout_ = time.time() + timeout_sec

    def _is_loop(self):
        return (not rospy.is_shutdown() and time.time() < self.timeout_)
    
    def _wait_service_ready(self):
        print("Wait for service...")
        self._set_payload_param_service = {}
        for controller_name in self._controller_list:
            if(controller_name != "left_arm_controller" and controller_name != "right_arm_controller"):
                continue
            rospy.wait_for_service(self._ns + controller_name + '/set_payload_param', timeout=3.0)
            service_name = self._ns + controller_name + '/set_payload_param'
            self._set_payload_param_service[controller_name] = rospy.ServiceProxy(service_name, SetPayloadParam)

    def _wait_first_receive(self):
        print("Wait for first receive...")
        self._set_timeout(3.0)
        while(self._is_loop()):
            time.sleep(0.1)
            loop_end = True
            self._publish_joint_states()
            for controller_name in self._controller_list:
                if(not self._first_recv_dynamics[controller_name] or not self._first_recv_joint_state[controller_name]):
                    loop_end = False
                    break
            if loop_end:
                return
        raise Exception("wait for first receive is failed.")

    def _torobo_dynamics_callback(self, torobo_dynamics, controller_name):
        if(not self._first_recv_dynamics[controller_name]):
            self._first_recv_dynamics[controller_name] = True
        self._torobo_dynamics[controller_name] = torobo_dynamics

    def _torobo_joint_state_callback(self, torobo_joint_state, controller_name):
        if(not self._first_recv_joint_state[controller_name]):
            self._first_recv_joint_state[controller_name] = True
        self._torobo_joint_state[controller_name] = torobo_joint_state
    
    def _publish_joint_states(self):
        self._joint_states_pub.publish(self._joint_states)
    
    def _set_joint_positions(self, joint_names, positions, publish=True):
        for (i, name) in enumerate(joint_names):
            if(not name in self._joint_states.name):
                continue
            idx = self._joint_states.name.index(name)
            self._joint_states.position[idx] = positions[i]
        self._publish_joint_states()

    def _print_joint_positions(self):
        for controller in self._controller_list:
            print("[Controller]: ", controller)
            print("  Pos: ", self._torobo_joint_state[controller].position)

    def _wait_joint_position_update(self, joint_names, expected_joint_positions):
        while self._is_loop():
            time.sleep(0.1)
            pass_count = 0
            for (i, joint_name) in enumerate(joint_names):
                for controller in self._controller_list:
                    if(not joint_name in self._torobo_joint_state[controller].name):
                        continue
                    idx = self._torobo_joint_state[controller].name.index(joint_name)
                    if(numpy.isclose(
                        self._torobo_joint_state[controller].position[idx],
                        expected_joint_positions[i])):
                        pass_count += 1
            if(pass_count == len(joint_names)):
                return
        raise Exception("wait for joint position update is failed.")

    def _wait_dynamics_update(self):
        for controller_name in self._controller_list:
            self._first_recv_dynamics[controller_name] = False
        while self._is_loop():
            time.sleep(0.1)
            pass_count = 0
            for controller_name in self._controller_list:
                if(self._first_recv_joint_state[controller_name]):
                    pass_count += 1
            if(pass_count == len(self._controller_list)):
                return
        raise Exception("wait for torobo_dynamics update is failed.")

    def test_CalcGravityTorque_Torso_90deg_ArmJ1J2_90deg(self):
        joint_names = ["left_arm/joint_1", "left_arm/joint_2", "right_arm/joint_1", "right_arm/joint_2"]
        positions = numpy.radians([90, 90, 90, 90])

        self._set_joint_positions(joint_names, positions)
        self._wait_joint_position_update(joint_names, positions)
        self._wait_dynamics_update()
        # self._print_joint_positions()

        assertAlmostEqual([17.374441805 , 0.0, 0.0, 0.0 ,0.0 , 0.0],
            self._torobo_dynamics["left_arm_controller"].gravity_compensation_effort)
        assertAlmostEqual([17.374441805 , 0.0, 0.0, 0.0 ,0.0 , 0.0],
            self._torobo_dynamics["right_arm_controller"].gravity_compensation_effort)
        assertAlmostEqual([0.0, -34.364561, 0.0, -0.22947561],
            self._torobo_dynamics["torso_head_controller"].gravity_compensation_effort)

    def test_CalcGravityTorque_Torso_90deg_ArmJ1J2_90deg_with_payload(self):
        joint_names = ["left_arm/joint_1", "left_arm/joint_2", "right_arm/joint_1", "right_arm/joint_2"]
        positions = numpy.radians([90, 90, 90, 90])

        self._set_joint_positions(joint_names, positions)
        self._wait_joint_position_update(joint_names, positions)

        # set payload
        response = self._set_payload_param_service["left_arm_controller"](
            "left_arm_controller", 1.0, [0, 0, 0.5], [])
        self.assertTrue(response.success)
        response = self._set_payload_param_service["right_arm_controller"](
            "right_arm_controller", 0.5, [0, 0, 0.2], [])
        self.assertTrue(response.success)

        self._wait_dynamics_update()

        assertAlmostEqual([27.887170604999, 0.0, 0.0, 0.0 ,0.0 , 0.0],
            self._torobo_dynamics["left_arm_controller"].gravity_compensation_effort)
        assertAlmostEqual([21.159808705, 0.0, 0.0, 0.0 ,0.0 , 0.0],
            self._torobo_dynamics["right_arm_controller"].gravity_compensation_effort)
        assertAlmostEqual([0.0, -48.66265669, 0.0, -0.22947561],
            self._torobo_dynamics["torso_head_controller"].gravity_compensation_effort)

if __name__ == '__main__':
    import rostest
    try:
        rostest.rosrun(PKG, NAME, TestToroboDynamics, sys.argv)
    except Exception, e:
        rospy.logerr("caught except: %s" % e)
    except:
        rospy.logerr("caught unknown except")