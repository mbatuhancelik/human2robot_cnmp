import cv2
import mediapipe as mp
import pyrealsense2 as rs
import numpy as np
import argparse
import os
import torch
import torch
import matplotlib.pyplot as plt

import utils
import dataset_utils
from data import FIR, map_to_unit_circle, JointLandmarkDataset
from environment import Environment

"""
Interactive demonstration loop
usage: 
    upon execution camera will directly open
    the circle around user's hand demonstrates internal state
        red: wait
        yellow: ready
        green: recording
    once the camera is activated, it will change between these states depending on time
    once the green state concludes, robot will execute the observed motion
    afterwards, program will ask whether to show landmark prediction
    press enter if you dont want
    type y and press enter if you want to see the predicted landmarks

    The last feature is enabled by giving -sl 1 argument

    small yellow circles demonstrate training locations, to help user to generate out of distribution data

    Arguments:

        -i : path to the model checkpoint
        -d : path to the training dataset
            this is required to get normalization factors from training data 

    Finally clause of final try catch phrase executes the model
    this part will need rework if training data structure has changed

"""
parser = argparse.ArgumentParser("Collect demonstration.")
parser.add_argument("-i", help="model path", type=str, required=True)
parser.add_argument("-d", help="train dataset", type=str, required=True)
args = parser.parse_args()
model = torch.load(args.i)
data_path = os.path.join("./data", args.d)
trainset = JointLandmarkDataset(os.path.join(data_path, "joints.pt"), os.path.join(data_path, "landmarks.pt"))
env = Environment(gui=1)
#arrays holding recording data
LMS_session = np.zeros((1500,33,  2))
images_session = np.zeros((1500, 720, 1280 , 3))
image_timestamps_session = np.zeros((1500, 1))
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
#init mp pose detector
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5)

for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)



if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 1280, 720  , rs.format.bgr8, 30)

# Start streaming
demmonstration = -1
pipeline.start(config)
input()
while True:
    demmonstration += 1
    # input("Start demonstration ?")
    try:
        prev_time = 1
        at_start = False
        at_start_time = None
        recording = False
        counter = 0
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
                
            ts = frames.get_timestamp()
            if at_start_time == None:
                at_start_time = ts
            ms = ts - prev_time
            print(f"fps: {1/ms* 1e3}, coutner = {counter}")
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            results = pose.process(color_image[:,:,[2,0,1]].copy())
            if not results.pose_landmarks:
                continue
            landmarks = utils.landmarkList_to_list(results.pose_landmarks.landmark)
            # if keyboard.is_pressed('p'):
            #     break
            # if keyboard.is_pressed('o'):
            #     at_start = True
            #     if at_start_time is None:
            #         at_start_time = frames.get_timestamp()
            if frames.get_timestamp() - at_start_time > 45e2:
                break
            if frames.get_timestamp() - at_start_time > 1e3:
                at_start = True
                if at_start_time is None:
                    at_start_time = frames.get_timestamp()
            color = (0,0,255)
            if at_start:
                if frames.get_timestamp() - at_start_time > 3e3:
                    recording = True
                else:
                    color = (0,255,255)
            if recording:
                color = color = (0,255,0)
            # draws a circle around the left hand
            cv2.circle(
                color_image,
                (
                    int(color_frame.width * landmarks[20][0]),
                    int(color_frame.height*  landmarks[20][1])
                ), 
                int(color_frame.width * 0.07), color, 2)
            # draws landmark annotations arround the body
            mp_drawing.draw_landmarks(
            color_image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
            # draws the initial point and circles representing robotic actions
            cv2.circle(color_image,(color_frame.width//4 ,color_frame.height//2), int(color_frame.width * 0.04), color, 1)
            for actionn in range(7):
                cv2.circle(color_image,(int(color_frame.width *(0.25 + 0.2 * np.cos(np.pi * actionn / 6))),int(color_frame.height*  (0.5 - 0.3 * np.sin(np.pi * actionn / 6)))), int(color_frame.width * 0.02), [0,255,255], 1)
            # cv2.circle(color_image,(color_frame.width//2 ,color_frame.height//2), int(color_frame.width * 0.05), color, 1)
            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)
            if recording:
                LMS_session[counter] = (landmarks).copy()
                images_session[counter] = (color_image).copy()
                image_timestamps_session[counter] = ts
                counter += 1
            prev_time = frames.get_timestamp()
            cv2.waitKey(1)
            cv2.imshow('RealSense', color_image)
    finally:
        # continues to next demonstration if nothing is recorded
        if counter == 0:
            continue
        # if recording is successful, applies the same preprocessing applied to the training data
        landmarks_dem = (torch.tensor(LMS_session[:counter]))
        landmarks_dem = landmarks_dem[:,20, :]
        timestamps_dem = torch.tensor(image_timestamps_session[:counter])
        timestamps_dem = (timestamps_dem - timestamps_dem[0])/(timestamps_dem[-1] - timestamps_dem[0])
        landmarks_dem = dataset_utils.normalize_camera_traj(timestamps_dem, landmarks_dem, 30)
        landmarks_dem = landmarks_dem.reshape((1,30,2))
        landmark = map_to_unit_circle(landmarks_dem[:,:,0], landmarks_dem[:,:,1], fac = trainset.fac.mean())[0]
        landmark = FIR(landmark)

        # after preprocessing, generate sample for the model and call forward
        # use none for joint observation
        # so joints are generated only using landmarks
        timestamps_dem = torch.arange(30).unsqueeze(1) / 29
        obs_timesteps = torch.randint(30, size=(5,1))[:,0]
        sample = {}
        landmark = torch.cat([timestamps_dem.reshape(1,30,1), landmark], dim = -1)
        obs_landmarks = landmark[: , [10,15,20,25,29], :] 
        target_joints = timestamps_dem.reshape((1,30,-1)) 
        target_landmarks = timestamps_dem.reshape((1,30,-1))
        obs = (None, obs_landmarks)
        target = (target_joints, target_landmarks)
        with torch.no_grad():
            mean, std, land_1 , land_std = model(obs, target,observation_mask=(None, None) )
        
        # gives the generated angles to the robot
        # god forgive me for making this so complicated
        pos = env.use_joint_series(mean.reshape((30,-1)).cpu(), sleep = True)
        pos = torch.tensor(pos)
        ds = (pos[-5:] - pos[:5]).mean(dim= 0)
        rob_angle = torch.arctan(torch.tensor(ds[2] / ds[1]))/torch.pi * 180 
        ds = (landmark[:5, 0] - landmark[0,-1])[0]
        hum_angle = torch.arctan((ds[2] / ds[1] * -1))/torch.pi * 180 
        print(rob_angle.item(), hum_angle.item())
        a = input("print plot?")
        if a == "y":
            plt.figure(figsize=(15,15))
            plt.plot(land_1[0,:,0].detach().cpu(),land_1[0,:,1].detach().cpu(), label="hand movement pred")
            plt.plot(landmark[0,:,1].cpu() ,landmark[0,:,2].cpu(), label="hand movement")
            plt.scatter(landmark[: , [5,10,15,20,25], 1].cpu() ,landmark[: , [5,10,15,20,25], 2].cpu()  )
            plt.legend()
            plt.show()

    
pipeline.stop()
