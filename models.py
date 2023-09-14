import torch
import torch.nn as nn
import torch.distributions as D


class CNP(nn.Module):
    def __init__(self, in_shape, hidden_size, num_hidden_layers, min_std=0.1):
        super(CNP, self).__init__()
        self.d_x = in_shape[0]
        self.d_y = in_shape[1]

        self.encoder = []
        self.encoder.append(nn.Linear(self.d_x + self.d_y, hidden_size))
        self.encoder.append(nn.ReLU())
        for _ in range(num_hidden_layers - 1):
            self.encoder.append(nn.Linear(hidden_size, hidden_size))
            self.encoder.append(nn.ReLU())
        self.encoder.append(nn.Linear(hidden_size, hidden_size))
        self.encoder = nn.Sequential(*self.encoder)

        self.query = []
        self.query.append(nn.Linear(hidden_size + self.d_x, hidden_size))
        self.query.append(nn.ReLU())
        for _ in range(num_hidden_layers - 1):
            self.query.append(nn.Linear(hidden_size, hidden_size))
            self.query.append(nn.ReLU())
        self.query.append(nn.Linear(hidden_size, 2 * self.d_y))
        self.query = nn.Sequential(*self.query)

        self.min_std = min_std

    def nll_loss(self, observation, target, target_truth, observation_mask=None, target_mask=None):
        '''
        The original negative log-likelihood loss for training CNP.

        Parameters
        ----------
        observation : torch.Tensor
            (n_batch, n_context, d_x+d_y) sized tensor that contains context
            points.
            d_x: the number of query dimensions
            d_y: the number of target dimensions.
        target : torch.Tensor
            (n_batch, n_target, d_x) sized tensor that contains query dimensions
            of target (query) points.
            d_x: the number of query dimensions.
            note: n_context and n_target does not need to be the same size.
        target_truth : torch.Tensor
            (n_batch, n_target, d_y) sized tensor that contains target
            dimensions (i.e., prediction dimensions) of target points.
            d_y: the number of target dimensions
        observation_mask : torch.Tensor
            (n_batch, n_context) sized tensor indicating which entries should be
            used in aggregation. Used for batch input.
        target_mask : torch.Tensor
            (n_batch, n_target) sized tensor indicating which entries should be
            used for loss calculation. Used for batch input.

        Returns
        -------
        loss : torch.Tensor (float)
            The NLL loss.
        '''
        mean_joint,std_joint,mean_landmark, std_landmark = self.forward(observation, target, observation_mask)
        dist_joint = D.Normal(mean_joint, std_joint)
        nll_joint = -dist_joint.log_prob(target_truth[0])
        dist_landmark = D.Normal(mean_landmark, std_landmark)
        nll_landmark = -dist_landmark.log_prob(target_truth[1])
        if target_mask is not None:
            # sum over the sequence (i.e. targets in the sequence)
            nll_masked = (nll_joint * target_mask[0].unsqueeze(2)).sum(dim=1)
            # compute the number of entries for each batch entry
            nll_norm = target_mask[0].sum(dim=1).unsqueeze(1)
            # first normalize, then take an average over the batch and dimensions
            loss = (nll_masked / nll_norm).mean()
            # sum over the sequence (i.e. targets in the sequence)
            nll_masked = (nll_landmark* target_mask[1].unsqueeze(2)).sum(dim=1)
            # compute the number of entries for each batch entry
            nll_norm = target_mask[1].sum(dim=1).unsqueeze(1)
            # first normalize, then take an average over the batch and dimensions
            loss += (nll_masked / nll_norm).mean()
        else:
            loss = nll_joint.mean() + nll_landmark.mean()
        return loss

    def forward(self, observation, target, observation_mask=None):
        '''
        Forward pass of CNP.

        Parameters
        ----------
        observation : torch.Tensor
            (n_batch, n_context, d_x+d_y) sized tensor where d_x is the number
            of the query dimensions, d_y is the number of target dimensions.
        target : torch.Tensor
            (n_batch, n_target, d_x) sized tensor where d_x is the number of
            query dimensions. n_context and n_target does not need to be the
            same size.
        observation_mask : torch.Tensor
            (n_batch, n_context) sized tensor indicating which entries should be
            used in aggregation.

        Returns
        -------
        mean : torch.Tensor
            (n_batch, n_target, d_y) sized tensor containing the mean
            prediction.
        std : torch.Tensor
            (n_batch, n_target, d_y) sized tensor containing the standard
            deviation prediction.
        '''
        h = self.encode(observation)
        r = self.aggregate(h, observation_mask=observation_mask)
        h_cat = self.concatenate(r, target)
        query_out = self.decode(h_cat)
        mean = query_out[..., :self.d_y]
        logstd = query_out[..., self.d_y:]
        std = torch.nn.functional.softplus(logstd) + self.min_std
        return mean, std

    def encode(self, observation):
        h = self.encoder(observation)
        return h

    def decode(self, h):
        o = self.query(h)
        return o

    def aggregate(self, h, observation_mask):
        # this operation is equivalent to taking mean but for
        # batched input with arbitrary lengths at each entry
        # the output should have (batch_size, dim) shape

        if observation_mask is not None:
            h = (h * observation_mask.unsqueeze(2)).sum(dim=1)  # mask unrelated entries and sum
            normalizer = observation_mask.sum(dim=1).unsqueeze(1)  # compute the number of entries for each batch entry
            r = h / normalizer  # normalize
        else:
            # if observation mask is none, we assume that all entries
            # in the batch has the same length
            r = h.mean(dim=1)
        return r

    def concatenate(self, r, target):
        num_target_points = target.shape[1]
        r = r.unsqueeze(1).repeat(1, num_target_points, 1)  # repeating the same r_avg for each target
        h_cat = torch.cat([r, target], dim=-1)
        return h_cat
"""
multi modal cnp for multi embodiment learning
"""
class MMCNP(CNP):
    def __init__(self, hidden_size, num_hidden_layers, min_std=0.1, latent_dim = 32):
        super(CNP, self).__init__()
        # did not bother using arguments for these
        #change these if landmark or timestep dimensions change
        #these are landmark shapes
        # d_x = timestep dimension
        # d_y = landmark dimension
        self.d_x = 1
        self.d_y = 2
        self.min_std = min_std
        self.latent_dim = latent_dim
        self.encoder_landmark = []
        self.encoder_landmark.append(nn.Linear(self.d_x + self.d_y, hidden_size))
        self.encoder_landmark.append(nn.ReLU())
        for _ in range(num_hidden_layers - 1):
            self.encoder_landmark.append(nn.Linear(hidden_size, hidden_size))
            self.encoder_landmark.append(nn.ReLU())
        self.encoder_landmark.append(nn.Linear(hidden_size, latent_dim))
        self.encoder_landmark = nn.Sequential(*self.encoder_landmark)
        self.query_landmark = []
        self.query_landmark.append(nn.Linear(latent_dim+ self.d_x, hidden_size))
        self.query_landmark.append(nn.ReLU())
        for _ in range(num_hidden_layers - 1):
            self.query_landmark.append(nn.Linear(hidden_size, hidden_size))
            self.query_landmark.append(nn.ReLU())
        self.query_landmark.append(nn.Linear(hidden_size, 2 * self.d_y))
        self.query_landmark = nn.Sequential(*self.query_landmark)
        # did not bother using arguments for these
        #change these if landmark or timestep dimensions change
        #these are landmark shapes
        # d_x = timestep dimension
        # d_y = joint dimension
        self.d_x = 1
        self.d_y = 5
        self.encoder_joint = []
        self.encoder_joint.append(nn.Linear(self.d_x + self.d_y, hidden_size))
        self.encoder_joint.append(nn.ReLU())
        for _ in range(num_hidden_layers - 1):
            self.encoder_joint.append(nn.Linear(hidden_size, hidden_size))
            self.encoder_joint.append(nn.ReLU())
        self.encoder_joint.append(nn.Linear(hidden_size, latent_dim))
        self.encoder_joint = nn.Sequential(*self.encoder_joint)


        self.query_joint = []
        self.query_joint.append(nn.Linear(latent_dim + self.d_x, hidden_size))
        self.query_joint.append(nn.ReLU())
        for _ in range(num_hidden_layers - 1):
            self.query_joint.append(nn.Linear(hidden_size, hidden_size))
            self.query_joint.append(nn.ReLU())
        self.query_joint.append(nn.Linear(hidden_size, 2 * self.d_y))
        self.query_joint = nn.Sequential(*self.query_joint)

    def forward(self, observation, target, observation_mask=None):
        obs_joint, obs_landmark = observation
        # model does not support different timesteps for different emboediments
        # you can generate 2 h_cat to support this

        # encode landmarks
        # generating landmarks from joints is not supported
        # so you must give landmarks for each forward operation
        target_timesteps, _ = target
        h_landmark = self.encoder_landmark(obs_landmark)
        r_landmark = self.aggregate(h_landmark, observation_mask=observation_mask[1])

        #encode joints
        if obs_joint != None:
        # if joints are not none, encode joints and blend 2 encodings
            h_joint = self.encoder_joint(obs_joint)
            r_joint = self.aggregate(h_joint, observation_mask=observation_mask[0])
            p = torch.rand((r_joint.shape[0], 1))
            r = r_joint * p + r_landmark * (1-p)
        else:
        # joints can be none, in this case, only landmarks are used to generate the latent 
            r = r_landmark

        # concatenate target timesteps to encoding
        h_cat = self.concatenate(r, target_timesteps)

        # decode the h_cat for each embodiment
        query_joint = self.query_joint(h_cat)
        query_landmark = self.query_landmark(h_cat)

        # divide the outputs to mean and std
        
        # dy is joint dimension, its important to store that since joints can be none
        mean_joint = query_joint[..., :self.d_y]
        logstd_joint = query_joint[..., self.d_y:]
        std_joint = torch.nn.functional.softplus(logstd_joint) + self.min_std
        
        mean_landmark = query_landmark[..., :obs_landmark.shape[-1]-1]
        logstd_landmark = query_landmark[..., obs_landmark.shape[-1]-1:]
        std_landmark = torch.nn.functional.softplus(logstd_landmark) + self.min_std
        
        return mean_joint,  std_joint, mean_landmark, std_landmark