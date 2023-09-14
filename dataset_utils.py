import torch
import numpy as np
import argparse
import os
import zipfile

import wandb
from data import JointLandmarkDataset
"""
utilities to manipulate on drive data
check main function for each functionality
"""
keys = ["joints", "landmarks", "timestamps"]
def merge_datasets(args):
    path = os.path.join("data", args.large)
    l_landmark = torch.load(os.path.join(path, "landmarks.pt"))
    l_joint = torch.load(os.path.join(path, "joints.pt"))
    l_timestep = torch.load(os.path.join(path, "timestamps.pt"))

    path = os.path.join("data", args.small)
    s_landmark = torch.load(os.path.join(path, "landmarks.pt"))
    s_joint = torch.load(os.path.join(path, "joints.pt"))
    s_timestep = torch.load(os.path.join(path, "timestamps.pt"))

    landmark= torch.concat([l_landmark, s_landmark], dim = 0)
    joint= torch.concat([l_joint, s_joint], dim = 0)
    timestep= torch.concat([l_timestep, s_timestep], dim = 0)

    args.o = os.path.join("./data", args.o)
    if not os.path.exists((args.o)):
        os.makedirs(args.o)
    torch.save(landmark, os.path.join(args.o, "landmarks.pt"))
    torch.save(joint, os.path.join(args.o, "joints.pt"))
    torch.save(timestep, os.path.join(args.o, "timestamps.pt"))

def normalize_camera_traj(timestamps, landmarks, normalized_size):
    out_shape = list(landmarks.shape)
    out_shape[0] =  normalized_size
    landmarks_norm = torch.zeros(out_shape)
    timesteps = torch.arange(normalized_size+1) / (normalized_size)
    for i, t in enumerate(timesteps.numpy()[1:]):
        t_high = (timestamps>=t).int().argmax()
        t_low = t_high - 1
        d1 = (t - timestamps[t_low])
        d2 = (timestamps[t_high]- t)
        landmarks_norm[i] =  (d1 * landmarks[t_low] +  d2* landmarks[t_high]) / (d1+d2)
    return landmarks_norm
def normalize_timestamps(timestamps):
    timestamps = (timestamps- timestamps[0])
    timestamps = timestamps / timestamps[-1]
    return timestamps
def merge_rolls(args):
    output_folder = os.path.join(f"{os.getcwd()}/data", args.o)
    fields = {}
    for key in keys:
        fields[key] = [torch.load(os.path.join(output_folder, f"{key}_{i}.pt")) for i in range(args.i)]
    fields["timestamps"] = [normalize_timestamps(ts) for ts in fields["timestamps"]]
    normalized_size = 30
    fields["landmarks"] = [normalize_camera_traj(timestamps, traj, normalized_size) for traj, timestamps in zip(fields["landmarks"], fields["timestamps"])]
    fields["joints"] = [normalize_camera_traj(torch.arange(480).unsqueeze(-1) / (480 - 1), traj, normalized_size) for traj in (fields["joints"])]
    fields["timestamps"] = [torch.arange(normalized_size) / (normalized_size - 1)for k in fields["timestamps"]]
    for key in fields:
        fields[key] = [k.unsqueeze(0) for k in fields[key]]
        fields[key] = torch.cat(fields[key], dim = 0)
        print(os.path.join(output_folder, f"{key}.pt"))
        torch.save(fields[key], os.path.join(output_folder, f"{key}.pt"))
    # for i in range(args.i):
    #     os.remove(os.path.join(output_folder, f"{key}_{i}.pt"))
def upload_dataset_to_wandb(name, path):
    with zipfile.ZipFile(f"{name}.zip", "w", zipfile.ZIP_DEFLATED) as zipf:
        for file in os.listdir(path):
            if file != ".DS_Store":
                zipf.write(os.path.join(path, file), arcname=file)
    wandb.init(project="multideepsym", entity="colorslab")
    artifact = wandb.Artifact(name, type="dataset")
    artifact.add_file(f"{name}.zip")
    wandb.log_artifact(artifact)
    os.remove(f"{name}.zip")


def get_dataset_from_wandb(name):
    artifact = wandb.use_artifact(f"colorslab/multideepsym/{name}:latest", type="dataset")
    artifact_dir = artifact.download()
    archive = zipfile.ZipFile(os.path.join(artifact_dir, f"{name}.zip"), "r")
    archive.extractall(os.path.join("data", name))
    archive.close()
    os.remove(os.path.join(artifact_dir, f"{name}.zip"))
if __name__ == "__main__":

    parser = argparse.ArgumentParser("See dataset metrics.")
    parser.add_argument("action", type=str)
    parser.add_argument("-o", help="dataset name", type=str)
    parser.add_argument("-s", "--small", help="smaller dataset", type=str)
    parser.add_argument("-l", "--large", help="appended dataset", type=str)
    parser.add_argument("-i", help="number of rolls", type=int)

    args = parser.parse_args()

    if args.action == "merge_datasets":
        """
        creates a dataset by merging 2 previously existing ones
        required args:
        -o : name of the output dataset 
        -s : name of dataset 1
        -l : name of dataset 2
        """
        merge_datasets(args)
    if args.action == "merge_rolls":
        """
        merges demonstrations generated during data collection to a dataset
        for each action performed during demonstration, 3 files will generated: 
        
        landmark_{1}.pt
        joints_{1}.pt
        timestamps_{1}.pt 

        1 being the number of demonstration

        this function applies some preprocessing, reduces to number of timestamps to 30 
        for each modality

        you need to change this function too if you change simulation frequency

        finally 3 files are produced:
        landmarks.pt
        joints.pt
        timestamps.pt

        which contain all demonstrations and cna be used to initialize dataset objects

        required args:
        -o : name of the output dataset 
        -s : name of dataset 1
        -l : name of dataset 2
        """
        merge_rolls(args)


    """
    wandb is not implemented yet, leaving these in case we implement it in future
    """
    if args.action == "upload":
        name = args.o
        path = os.path.join("./data", name)
        upload_dataset_to_wandb(name, path)
    if args.action == "download":
        wandb.init(project="multideepsym", entity="colorslab")
        get_dataset_from_wandb(args.o)