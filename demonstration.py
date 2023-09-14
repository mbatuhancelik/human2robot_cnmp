import cv2
import mediapipe as mp
import pyrealsense2 as rs
import numpy as np
import argparse
import os
import torch
import keyboard
import torch
"""
collects data 
training loop: 
    robot performs
    human imitates

    one robot concludes demonstration, camera will activate
    you will see one yellow circle in the middle and smaller ones around the perimeter
    these show angles that are used for robot actions

    at first practice your action so that you can generate a clean trajectory

    then press o to start initial phase, circle around your hand is yellow in this phase
    recording will start when the circle around your hand turns green
    demonstrate the action
    press p to stop recording

    this concludes a demonstration session
    program will ask you to start the robot demonstration again
    once all actions are demonstrated, the program exits


    Raw data generated from each session will be saved to output folder in seperate files
    use dataset utils to perform preprocessing when all demonstrations are collected

    This process will generate a dataset containing one demonstration for each action
    if you want multiple demonstration for each action, 
    generate multiple datasets and merge them using dataset utils

"""
"""
this phase adds a phase difference to robots actions
given phase 1, robot will add 1 radian to its target distance
"""
phase = 3.14/12


from utils import landmarkList_to_list
from environment import Environment
parser = argparse.ArgumentParser("Collect demonstration.")
parser.add_argument("-o", help="output folder", type=str, required=True)
args = parser.parse_args()
args.o = os.path.join("./data", args.o)
if not os.path.exists((args.o)):
    os.makedirs(args.o)

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
env =  Environment(gui=1)
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
    # this is used for D series cameras
    # set resolution here
    # notice that image format is bgr, this will change to rgb when using mediapipe 
    config.enable_stream(rs.stream.color, 1280, 720  , rs.format.bgr8, 30)

# Start streaming
demmonstration = -1
pipeline.start(config)
for action in range(7):
    demmonstration += 1
    env.reset()
    # wait user input to start robot demonstration
    input("Start demonstration ?")
    jp = env.step(action, phase)
    joints, pos = torch.tensor(jp[0]), torch.tensor(jp[1])
    # from now on, collect human data inside try 
    # finalize case will save the collected data
    # so, collected data will be saved if the counter starts, check the sanity of the collected data upon collection
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
            landmarks = landmarkList_to_list(results.pose_landmarks.landmark)
            """
            if you want to do this by timing, there is an example under interactive_test.py
            """
            # press p to stop
            if keyboard.is_pressed('p'):
                break
            # press o to start recording sequence
            if keyboard.is_pressed('o'):
                at_start = True
                if at_start_time is None:
                    at_start_time = frames.get_timestamp()
            color = (0,0,255)
            # if some time has passed since the keydown on o key, recording starts
            if at_start:
                if frames.get_timestamp() - at_start_time > 3e3:
                    recording = True
                else:
                    color = (0,255,255)
            if recording:
                color = color = (0,255,0)
            # draw the circle around users hand
            cv2.circle(
                color_image,
                (
                    int(color_frame.width * landmarks[20][0]),
                    int(color_frame.height*  landmarks[20][1])
                ), 
                int(color_frame.width * 0.04), color, 2)
            # draw landmarks
            mp_drawing.draw_landmarks(
            color_image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
            # draw action indicators
            cv2.circle(color_image,(color_frame.width//4 ,color_frame.height//2), int(color_frame.width * 0.02), color, 1)
            for actionn in range(7):
                cv2.circle(color_image,(int(color_frame.width *(0.25 + 0.2 * np.cos(np.pi * actionn / 6 + phase))),int(color_frame.height*  (0.5 - 0.3 * np.sin(np.pi * actionn / 6+ phase)))), int(color_frame.width * 0.02), [0,255,255], 1)
            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)
            # save landmarks and images when recording
            if recording:
                LMS_session[counter] = (landmarks).copy()
                images_session[counter] = (color_image).copy()
                image_timestamps_session[counter] = ts
                counter += 1
            prev_time = frames.get_timestamp()
            cv2.waitKey(1)

    finally:
        # saves collected RAW data
        # use dataset_utils to perform preprocessing
        torch.save(torch.tensor(LMS_session[:counter]), os.path.join(args.o, f"landmarks_{demmonstration}.pt"))
        torch.save(torch.tensor(images_session[:counter]), os.path.join(args.o, f"images_{demmonstration}.pt"))
        torch.save(torch.tensor(image_timestamps_session[:counter]), os.path.join(args.o, f"timestamps_{demmonstration}.pt"))
        torch.save(joints, os.path.join(args.o, f"joints_{demmonstration}.pt"))
        torch.save(pos, os.path.join(args.o, f"pos_{demmonstration}.pt"))
        # Stop streaming
    cv2.imshow('RealSense', color_image)
pipeline.stop()
