#!/usr/bin/env python
# -*- coding: utf-8 -*-

PKG = 'torobo_driver'
NAME = 'test_torobo_easy_command'

import sys
import time
import unittest
import numpy
import rospy

from torobo_msgs.msg import ToroboJointState
from torobo_driver import torobo_easy_command

# for Test const value
SERVO_OFF = 0
SERVO_ON = 2
ERROR_NOT_OCCURED = 0
ERROR_OCCURED = 3
TRAJ_COMPLETE = 4

class TestToroboEasyCommand(unittest.TestCase):
    def __init__(self, *args):
        super(TestToroboEasyCommand, self).__init__(*args)
        rospy.init_node(NAME, anonymous=True)
        self._ns = rospy.get_namespace()
        self._controller_name = rospy.get_param("~controller_name")
        self._first_recv = False
        self._torobo_joint_state = ToroboJointState()
        self._torobo_joint_state_sub = rospy.Subscriber(self._ns + self._controller_name + "/torobo_joint_state", ToroboJointState, self._torobo_joint_state_callback)
        self._wait_service_ready()
        self._wait_first_receive()
        self._joint_size = len(self._torobo_joint_state.name)
        print("Ready.")

    def setUp(self):
        self._set_timeout(3.0)

    def tearDown(self):
        pass

    def _set_timeout(self, timeout_sec=3.0):
        self.timeout_ = time.time() + timeout_sec

    def _is_loop(self):
        return (not rospy.is_shutdown() and time.time() < self.timeout_)
    
    def _wait_service_ready(self):
        print("Wait for service...")
        rospy.wait_for_service(self._ns + self._controller_name + '/send_common_command', timeout=3.0)

    def _wait_first_receive(self):
        print("Wait for first receive...")
        self._set_timeout(3.0)
        while(self._is_loop()):
            if(self._first_recv):
                return
        raise Exception("wait for first receive is failed.")

    def _torobo_joint_state_callback(self, torobo_joint_state):
        if(not self._first_recv):
            self._first_recv = True
        self._torobo_joint_state = torobo_joint_state

    def _send_torobo_common_command_with_receive_check(self, command_text, timeout=3.0):
        timestamp = self._torobo_joint_state.hostTimeStamp
        torobo_easy_command.SendEasyCommandText(ns=self._ns + self._controller_name, text=command_text)

        self._set_timeout(timeout)
        while self._is_loop():
            if(timestamp != self._torobo_joint_state.hostTimeStamp):
                return True
            time.sleep(0.1)
        rospy.logerr("send common command [%s] is timeout." % command_text)
        return False

    def _set_error_status(self, joint_number, error_number):
        text = "debug " + str(joint_number) + " 255 " + str(error_number)
        self._send_torobo_common_command_with_receive_check(text)

    def _set_control_mode(self, joint_number, control_mode_number):
        text = "r " + str(joint_number) + " " + str(control_mode_number)
        self._send_torobo_common_command_with_receive_check(text)

    def assertAllJointSameValue(self, expectedValue, container):
        expected_tuple = tuple([expectedValue for i in range(self._joint_size)])
        self.assertTupleEqual(expected_tuple, container)

    def assertAlmostEqual(self, expected, actual, decimal=7):
        numpy.testing.assert_almost_equal(expected, actual, decimal)

    def test_reset_all(self):
        # set error
        expected_error_number = 1
        self._set_error_status(joint_number="all", error_number=expected_error_number)

        # check error is occured
        self.assertAllJointSameValue(expected_error_number, self._torobo_joint_state.errorWarningStatus)
        self.assertAllJointSameValue(ERROR_OCCURED, self._torobo_joint_state.systemMode)

        # send reset
        self._send_torobo_common_command_with_receive_check("reset all")

        # assert no error
        self.assertAllJointSameValue(0, self._torobo_joint_state.errorWarningStatus)
        self.assertAllJointSameValue(ERROR_NOT_OCCURED, self._torobo_joint_state.systemMode)

    def test_change_control_mode(self):
        # r 4 5
        self._set_control_mode(joint_number=4, control_mode_number=5)
        self.assertEqual(5, self._torobo_joint_state.ctrlMode[3])

        # r 4 0
        self._set_control_mode(joint_number=4, control_mode_number=0)
        self.assertEqual(0, self._torobo_joint_state.ctrlMode[3])

        # r all 5
        self._set_control_mode(joint_number="all", control_mode_number=5)
        self.assertAllJointSameValue(5, self._torobo_joint_state.ctrlMode)

        # r all 0
        self._set_control_mode(joint_number="all", control_mode_number=0)
        self.assertAllJointSameValue(0, self._torobo_joint_state.ctrlMode)

    def test_servo_on_off(self):
        # s 4 1
        self._send_torobo_common_command_with_receive_check("s 4 1")
        self.assertEqual(SERVO_ON, self._torobo_joint_state.systemMode[3])

        # s 4 0
        self._send_torobo_common_command_with_receive_check("s 4 0")
        self.assertEqual(SERVO_OFF, self._torobo_joint_state.systemMode[3])

        # s all 1
        self._send_torobo_common_command_with_receive_check("s all 1")
        self.assertAllJointSameValue(SERVO_ON, self._torobo_joint_state.systemMode)

        # s all 0
        self._send_torobo_common_command_with_receive_check("s all 0")
        self.assertAllJointSameValue(SERVO_OFF, self._torobo_joint_state.systemMode)

    def test_trajectory_control(self):
        # set trajectory control mode all joints
        self._set_control_mode(joint_number="all", control_mode_number=0)
        # servo on joint 4
        self._send_torobo_common_command_with_receive_check("s 4 1")

        # check pre status
        self.assertEqual(SERVO_ON, self._torobo_joint_state.systemMode[3])
        self.assertEqual(0, self._torobo_joint_state.ctrlMode[3])
        self.assertEqual(0, self._torobo_joint_state.trjViaRemain[3])
        self.assertAlmostEqual(0.0, self._torobo_joint_state.position[3])

        # tpts 4 90 5
        self._send_torobo_common_command_with_receive_check("tpts 4 90 5")
        # check via remain is 1
        self.assertEqual(1, self._torobo_joint_state.trjViaRemain[3])

        # ts 4
        self._send_torobo_common_command_with_receive_check("ts 4")
        self._set_timeout(7.0)
        while self._is_loop():
            if(TRAJ_COMPLETE == self._torobo_joint_state.trjStatus[3]):
                break
            print("wait for trajectory control complete...: now:%f, goal:%f"
             % (numpy.degrees(self._torobo_joint_state.position[3]), 90.0))
            time.sleep(0.5)
        
        # check trajectory control complete
        self.assertEqual(TRAJ_COMPLETE, self._torobo_joint_state.trjStatus[3])
        self.assertAlmostEqual(numpy.radians(90.0), self._torobo_joint_state.position[3])


if __name__ == '__main__':
    import rostest
    try:
        rostest.rosrun(PKG, NAME, TestToroboEasyCommand, sys.argv)
    except Exception, e:
        rospy.logerr("caught except: %s" % e)
    except:
        rospy.logerr("caught unknown except")