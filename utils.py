import numpy as np
import mediapipe as mp
import cv2 
import yaml
import wandb
import os
import torch


import models

def landmarkList_to_list(landmarkList):
    landmarks = np.zeros((33, 2))
    for i, l in enumerate(landmarkList):
        landmarks[i] = np.array([l.x, l.y])
    return landmarks

def connect(gui=1):
    import pkgutil
    from pybullet_utils import bullet_client
    if gui:
        p = bullet_client.BulletClient(connection_mode=bullet_client.pybullet.GUI)
    else:
        p = bullet_client.BulletClient(connection_mode=bullet_client.pybullet.DIRECT)
        egl = pkgutil.get_loader("eglRenderer")
        if (egl):
            p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
        else:
            p.loadPlugin("eglRendererPlugin")
    return p


def parse_and_init(args):
    with open(args.config, "r") as f:
        config = yaml.safe_load(f)
    # init run
    run = wandb.init(project="h2robot", entity="colorslab", config=config)
    # create a save folder if not exists
    save_folder = run.config["save_folder"]
    os.makedirs(save_folder, exist_ok=True)
    # also save the config file in the save folder
    with open(os.path.join(save_folder, "config.yaml"), "w") as f:
        yaml.dump(config, f)

    # download and extract dataset if not exissts
    data_path = os.path.join("data", config["dataset_name"])
    if not os.path.exists(data_path):
        get_dataset_from_wandb(config["dataset_name"])

    return wandb.config

def get_dataset_from_wandb(dataset):
    #implement when you have complete datasets
    raise Exception("Invalid dataset")
def print_module(module, name, space):
    L = len(name)
    line = " "*space+"-"*(L+4)
    print(line)
    print(" "*space+"  "+name+"  ")
    print(line)
    module_str = module.__repr__()
    print("\n".join([" "*space+mstr for mstr in module_str.split("\n")]))
def get_parameter_count(model):
    total_num = 0
    for param in model.parameters():
        total_num += param.shape.numel()
    return total_num
def load_model(id, prefix):
    run = wandb.init(entity="colorslab", project="h2robot", resume="must", id=id)
    model = create_model_from_config(run.config)
    model.load("_best")
    return model
def wandb_finalize():
    wandb.finish()