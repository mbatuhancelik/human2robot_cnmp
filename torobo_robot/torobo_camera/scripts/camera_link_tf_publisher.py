#!/usr/bin/env python
# -*- coding: utf-8 -*-


import math
import numpy as np
import rospy
import tf
import tf2_ros
import cv2
import yaml
import argparse

from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError


class CameraLinkTfPublisher:

    def __init__(self):
        rospy.init_node("camera_link_tf_publisher_node", anonymous=True)
        rospy.sleep(1.0)
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

    def do_and_spin(self, filepath):
        # load pose data
        with open(filepath, "r") as f:
            msg = yaml.load(f)

        quat = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]
        euler = tf.transformations.euler_from_quaternion(quat)
        msg.header.stamp = rospy.Time.now()
        rospy.loginfo(msg)
        rospy.loginfo(np.degrees(euler))

        # broadcast tf_static
        staticBroadcaster = tf2_ros.StaticTransformBroadcaster()
        rospy.sleep(1.0)
        staticBroadcaster.sendTransform([msg])

        rospy.sleep(1.0)
        #rospy.spin()


if __name__ == '__main__':

    def parse_arguments(args):
        parser = argparse.ArgumentParser()
        parser.add_argument("--camera_link_tf_data", type=str, default="")
        return parser.parse_args(args)
    args = parse_arguments(rospy.myargv()[1:])


    cpe = CameraLinkTfPublisher()
    cpe.do_and_spin(args.camera_link_tf_data)


