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
import os

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError

PREFIX_ESTIMATED = 'ESTIMATED_'

class CameraLinkTfEstimator:

    def __init__(self):
        rospy.init_node("camera_link_tf_estimater_node", anonymous=True)
        rospy.sleep(1)
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

    def get_image(self, color_image_topic):
        msg = rospy.wait_for_message(color_image_topic, Image)
        img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        return img

    def get_camera_info(self, camera_info_topic):
        msg = rospy.wait_for_message(camera_info_topic, CameraInfo)
        return msg

    def estimate_pose_marker(self, img, camera_info, square_size, square_marker_ratio):
        intrinsic_matrix = np.array(camera_info.K).reshape((3, 3))
        intrinsic_distortion = np.array([camera_info.D])
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(img, dictionary)
        diamondCorners, diamondIds = cv2.aruco.detectCharucoDiamond(img, corners, ids, square_marker_ratio)
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
        cv2.aruco.drawDetectedDiamonds(img, diamondCorners, diamondIds)
        ret = cv2.aruco.estimatePoseSingleMarkers(diamondCorners, square_size, intrinsic_matrix, intrinsic_distortion)
        rvec = ret[0]
        tvec = ret[1]
        cv2.aruco.drawAxis(img, intrinsic_matrix, intrinsic_distortion, rvec, tvec, 0.1)
        return rvec, tvec, img

    def create_tf_from_marker_to_estimated_camera_color_optical_frame(self, marker_frame_id, camera_color_optical_frame_id, rvec, tvec):
        rot, _ = cv2.Rodrigues(rvec)
        tvec = tvec[0]
        inv_rot = rot.T
        inv_tvec = -np.matmul(rot.T, tvec.T)
        euler = tf.transformations.euler_from_matrix(inv_rot, axes='sxyz')
        quat = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2], axes='sxyz')
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.transform.translation.x = inv_tvec[0, 0]
        msg.transform.translation.y = inv_tvec[1, 0]
        msg.transform.translation.z = inv_tvec[2, 0]
        msg.transform.rotation.x = quat[0]
        msg.transform.rotation.y = quat[1]
        msg.transform.rotation.z = quat[2]
        msg.transform.rotation.w = quat[3]
        msg.child_frame_id = PREFIX_ESTIMATED + camera_color_optical_frame_id
        msg.header.frame_id = marker_frame_id
        return msg

    def create_tf_from_estimated_camera_color_optical_frame_to_estimated_camera_link(self, camera_color_optical_frame_id, camera_link_id):
        msg = self.tfBuffer.lookup_transform(camera_color_optical_frame_id, camera_link_id, rospy.Time.now(), rospy.Duration(1.0))
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = PREFIX_ESTIMATED + camera_color_optical_frame_id
        msg.child_frame_id = PREFIX_ESTIMATED + camera_link_id
        return msg

    def broadcast_tf_static(self, marker_tf, camera_color_optical_frame_id, camera_link_id, rvec, tvec):
        marker_frame_id = marker_tf.child_frame_id
        staticBroadcaster = tf2_ros.StaticTransformBroadcaster()
        rospy.sleep(1.0)
        trans1 = marker_tf
        trans2 = self.create_tf_from_marker_to_estimated_camera_color_optical_frame(
            marker_frame_id, camera_color_optical_frame_id, rvec, tvec)
        trans3 = self.create_tf_from_estimated_camera_color_optical_frame_to_estimated_camera_link(
            camera_color_optical_frame_id, camera_link_id)
        staticBroadcaster.sendTransform([trans1, trans2, trans3])

    def create_camera_link_tf(self, camera_base_id, camera_link_id):
        msg = self.tfBuffer.lookup_transform(
            camera_base_id, PREFIX_ESTIMATED + camera_link_id, rospy.Time.now(), rospy.Duration(1.0))
        msg.child_frame_id = camera_link_id
        print msg
        return msg


def main(args):
    with open(args.config, 'r') as f:
        config = yaml.load(f)
        square_size = config['marker']['square_size']
        square_marker_ratio = config['marker']['square_marker_ratio']
        color_image_topic = config['camera']['color_image_topic']
        camera_info_topic = config['camera']['camera_info_topic']
        camera_base_id = config['camera']['camera_base_id']
        camera_link_id = config['camera']['camera_link_id']
        camera_color_optical_frame_id = config['camera']['camera_color_optical_frame_id']

    with open(args.marker_tf_data) as f:
        marker_tf = yaml.load(f)
        marker_frame_id = marker_tf.child_frame_id

    cpe = CameraLinkTfEstimator()

    # get color image
    img = cpe.get_image(color_image_topic)

    # get camera info
    camera_info = cpe.get_camera_info(camera_info_topic)

    # estimate tf of pose marker
    rvec, tvec, img = cpe.estimate_pose_marker(img, camera_info, square_size, square_marker_ratio)
    estimated_pose_marker_path = os.path.join(os.path.dirname(args.config), "estimated_pose_marker.png")
    cv2.imwrite(estimated_pose_marker_path, img)
    rospy.loginfo("output estimated pose marker image data : " + estimated_pose_marker_path)

    # estimate tf of camera_link
    cpe.broadcast_tf_static(marker_tf, camera_color_optical_frame_id, camera_link_id, rvec, tvec)
    camera_link_tf = cpe.create_camera_link_tf(camera_base_id, camera_link_id)

    # dump tf of camera_link
    with open(args.camera_link_tf_data, "w") as f:
        f.write(yaml.dump(camera_link_tf))
        rospy.loginfo("output calibrated camera_link pose data : " + args.camera_link_tf_data)


if __name__ == '__main__':
    def parse_arguments(args):
        parser = argparse.ArgumentParser()
        parser.add_argument("--config", type=str, default="")
        parser.add_argument("--marker_tf_data", type=str, default="")
        parser.add_argument("--camera_link_tf_data", type=str, default="")
        return parser.parse_args(args)
    args = parse_arguments(rospy.myargv()[1:])
    main(args)
