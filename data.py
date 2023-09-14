import torch
from torch.nn.utils.rnn import pad_sequence
import numpy as np
"""
this module performs preprocessing on colledted data and used to 
generate pytorch datasets

Check JointLandmarkDataset which is used alongside MMCNP model
"""

def map_to_unit_circle( x, y, fac = None):
        """
        normalizes the collected landmark trajectories to unit circle
        during testing, use the average fac coming from the training data
        """
        x = x -  x[:,0, ...].unsqueeze(-1)
        y = y - y[:, 0, ...].unsqueeze(-1)
        if fac == None:
            fac = x**2 + y **2
            fac = (fac).max(dim = -1)[0]
            fac = torch.sqrt(fac)
        x = x/ fac.unsqueeze(-1)
        y = y/fac.unsqueeze(-1)
        xy = torch.cat([x.unsqueeze(-1),y.unsqueeze(-1)], dim = -1)
        return xy, fac
def FIR(landmark) : 
        """
        applies a low pass filter to landmarks
        depending on the number of steps, filter coefficients may change

        number of timesteps are hardcoded, filter properties may change with number of timesteps
        landmark dimension:
            [action, timesteps, landmark_shape]
        """
        for l in range(landmark.shape[-1 ]):
            for act in range(landmark.shape[0]):
                z = torch.cat([landmark[[act], :, l], landmark[act, [-1], l].reshape(1,1) ], dim = -1)
                for i in range(6):
                    z = torch.cat([z, landmark[act, [-1], l].reshape(1,1) ], dim = -1)
                k = torch.nn.functional.conv1d(z, torch.tensor([0.2, 0.2, 0.2,0.2,0.2]).reshape(1,1,5), bias=None, stride=1, padding='same', dilation=1, groups=1)
                #depending on num timesteps, this assignment may change
                landmark[act, 5:, l] = k[0, 5:30]
        return landmark

class CNPDemonstrationDataset:
    def __init__(self) -> None:
        self.N = 0
        self.data = []

    def get_sample(self, batch_size=1, max_context=10, max_target=10, R_g= None):
        context_all, target_all, context_mask, target_mask = [], [], [], []
        for _ in range(batch_size):
            n_context = torch.randint(1, max_context, ())
            n_target = torch.randint(1, max_target, ())
            idx = torch.randint(0, self.N, ())
            traj = self.data[idx]
            if R_g is None:
                R = torch.randperm(traj.shape[0])
            else:
                R = R_g[_]
            context = traj[R[:n_context]]
            target = traj[R[:(n_context+n_target)]]
            context_all.append(context)
            target_all.append(target)
            context_mask.append(torch.ones(context.shape[0]))
            target_mask.append(torch.ones(target.shape[0]))
        context_all = pad_sequence(context_all, batch_first=True)
        target_all = pad_sequence(target_all, batch_first=True)
        context_mask = pad_sequence(context_mask, batch_first=True)
        target_mask = pad_sequence(target_mask, batch_first=True)
        return context_all, target_all, context_mask, target_mask
class LandmarkDataset(CNPDemonstrationDataset):
    def __init__(self, data_path):
        landmark = torch.load(data_path)
        n_traj, n_length, n_landmark, n_dim = landmark.shape
        self.N = n_traj
        self.data = landmark.reshape(n_traj, n_length, n_landmark*n_dim)
        time = torch.linspace(0, 1, n_length).repeat(n_traj, 1).unsqueeze(2)
        self.data = torch.cat([time, self.data], dim=-1)


class JointDataset(CNPDemonstrationDataset):
    def __init__(self, data_path):
        self.data = torch.load(data_path)
        self.N = self.data.shape[0]
        time = torch.linspace(0, 1, self.data.shape[1]).repeat(self.N, 1).unsqueeze(2)
        self.data = torch.cat([time, self.data], dim=-1)


"""
dataset definition containing both joints and landmarks 
phase is used when synthetic data is used
use phase 0 for training data and some other value for validation data
"""
class JointLandmarkDataset(CNPDemonstrationDataset):
    def __init__(self, joint_path, landmark_path, phase = 0, synthetic = False):
        
        if synthetic:
            # generate synthetic data
            landmark= torch.zeros((6, 30,1 , 2)) 
            self.joint =  torch.zeros((6,30,5))
            for i in range(landmark.shape[0]):
                ts = torch.arange(landmark.shape[1]) /landmark.shape[1]
                act = i % 7
                landmark[i,: ,0, 0] = ts * np.cos(act * np.pi/6 + phase)
                landmark[i,: ,0, 1] = ts * np.sin(act * np.pi/6 + phase)

            for i in range(landmark.shape[0]):
                for k in range(self.joint.shape[-1]):
                    self.joint[i,:,k] = np.sin(ts *10 * np.pi/6 +act * np.pi/8 +phase + k)
        else:
            landmark = torch.load(landmark_path)[:,:,20,:]
            self.joint= torch.load(joint_path)[:,:,:5]
            landmark = FIR(landmark)
            landmark, self.fac = map_to_unit_circle(landmark[:,:,0], landmark[:,:,1])
            landmark = landmark.unsqueeze(-2)
            


        
        n_traj, n_length, n_landmark, n_dim = landmark.shape
        self.N = n_traj
        self.data = landmark.reshape(n_traj, n_length, n_landmark*n_dim)
        time = torch.linspace(0, 1, n_length).repeat(n_traj, 1).unsqueeze(2)
        self.landmark = torch.cat([time, self.data], dim=-1)
        # Currenytly 5 joints are used out of 7
        self.N = self.data.shape[0]
        self.idx = 0
        time = torch.linspace(0, 1, self.data.shape[1]).repeat(self.N, 1).unsqueeze(2)
        self.joint = torch.cat([time, self.joint], dim=-1)

    
    """
    generates samples for training
    each sample consisting of a 4 tuple:
        each element in this 4 tuple consists of a 2 tuple: one for landmark and one for joints

        elements are in the following:
            (joint_context_all, landmark_context_all), : observation data, timestep at first index
            (joint_target_all, landmark_target_all),: prediction timesteps for each embodiment
            (joint_context_mask, landmark_context_mask), : observation mask for each embodiment
            (joint_target_mask, landmark_target_mask) : prediction mask for each embodiment
    """
    def get_sample(self, batch_size=1, max_context=10, max_target=10, R_g= None):
        joint_context_all, joint_target_all, joint_context_mask, joint_target_mask = [], [], [], []
        landmark_context_all, landmark_target_all, landmark_context_mask, landmark_target_mask = [], [], [], []
        for _ in range(batch_size):
            # randomly select context and target sizes
            n_context = torch.randint(1, max_context, ())
            n_target = torch.randint(1, max_target, ())
            
            # select an action
            # increment id with each sample, or randomly select

            # idx = torch.randint(0, self.N, ())
            idx = self.idx
            self.idx = (idx + 1) %self.landmark.shape[0]

            joint_traj = self.joint[idx]
            landmark_traj = self.landmark[idx]
            R = torch.randperm(joint_traj.shape[0])
            context_joint = joint_traj[R[:n_context]]
            target_joint = joint_traj[R[:(n_context+n_target)]]
            context_landmark= landmark_traj[R[:n_context]]
            target_landmark = landmark_traj[R[:(n_context+n_target)]]
            joint_context_all.append(context_joint)
            joint_target_all.append(target_joint)
            joint_context_mask.append(torch.ones(context_joint.shape[0]))
            joint_target_mask.append(torch.ones(target_joint.shape[0]))

            landmark_context_all.append(context_landmark)
            landmark_target_all.append(target_landmark)
            landmark_context_mask.append(torch.ones(context_landmark.shape[0]))
            landmark_target_mask.append(torch.ones(target_landmark.shape[0]))
        joint_context_all = pad_sequence(joint_context_all, batch_first=True)
        joint_target_all = pad_sequence(joint_target_all, batch_first=True)
        joint_context_mask = pad_sequence(joint_context_mask, batch_first=True)
        joint_target_mask = pad_sequence(joint_target_mask, batch_first=True)

        landmark_context_all = pad_sequence(landmark_context_all, batch_first=True)
        landmark_target_all = pad_sequence(landmark_target_all, batch_first=True)
        landmark_context_mask = pad_sequence(landmark_context_mask, batch_first=True)
        landmark_target_mask = pad_sequence(landmark_target_mask, batch_first=True)
        return (joint_context_all, landmark_context_all), (joint_target_all, landmark_target_all),\
        (joint_context_mask, landmark_context_mask), (joint_target_mask, landmark_target_mask)




def unequal_collate(batch):
    context_all, target_all, context_mask, target_mask = [], [], [], []
    for context, target in batch:
        context_all.append(context)
        target_all.append(target)
        context_mask.append(torch.ones(context.shape[0]))
        target_mask.append(torch.ones(target.shape[0]))
    context_all = pad_sequence(context_all, batch_first=True)
    target_all = pad_sequence(target_all, batch_first=True)
    context_mask = pad_sequence(context_mask, batch_first=True)
    target_mask = pad_sequence(target_mask, batch_first=True)
    return context_all, target_all, context_mask, target_mask
