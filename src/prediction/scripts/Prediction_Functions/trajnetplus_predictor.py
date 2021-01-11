#!/usr/bin/env python3
# VITA, EPFL
import os
from collections import OrderedDict, defaultdict
import argparse
import pickle

import numpy as np
import scipy
import torch

import trajnetplusplustools
import trajnetbaselines

## Parallel Compute
import multiprocessing
from joblib import Parallel, delayed
from tqdm import tqdm

from trajnetplusplustools.data import SceneRow, TrackRow

class TrajNetPredictor():

    def __init__(self, dt=1.0, pred_horizon=10.0, obs_length=5, model_name="LSTM", model="/home/cconejob/trajnet++/trajnetplusplusbaselines/trajnetbaselines/lstm/lstm_directional_one_12_6_rerun.pkl", tag=2):
        self.unimodal = True
        self.modes = 3
        self.model_name = model_name
        self.device = torch.device('cpu')
        self.frame_number = 1
        self.scene_number = 1
        self.obs_length = obs_length
        self.dt = dt
        self.pred_length = int(pred_horizon/self.dt)
        self.fps = 1./dt
        self.tag = tag
        self.tracks_by_frame = defaultdict(list)
        self.scenes_by_id = dict()
        
        if 'SGAN' in self.model_name:
            self.predictor = trajnetbaselines.sgan.SGANPredictor.load(model)
            self.goal_flag = self.predictor.model.generator.goal_flag

        else:
            self.predictor = trajnetbaselines.lstm.LSTMPredictor.load(model)
            self.goal_flag = self.predictor.model.goal_flag

        self.predictor.model.to(self.device)
        
    def prediction_function(self, position_present_past):
        scene = {}
        track = {}
        self.frame_number += 1

        for idx,detection in enumerate(position_present_past):
            scene = {"id": self.frame_number, "p": detection[0][2], "s": self.frame_number, "e": self.frame_number+self.obs_length, "fps": self.fps, "tag": self.tag}
            
            row = SceneRow(scene['id'], scene['p'], scene['s'], scene['e'], scene['fps'], scene['tag'])
            self.scenes_by_id[row.scene] = row
            
            for i in range(len(detection)):
                track = {"f": self.frame_number + i, "p": detection[i][2], "x": detection[i][0], "y": detection[i][1], "pred_number": min(3, len(position_present_past)), "scene_id": self.frame_number}
        
                row = TrackRow(track['f'], track['p'], track['x'], track['y'], track['pred_number'], track['scene_id'])
                self.tracks_by_frame[row.frame].append(row)

        pred_list = self.get_predictions(self.scenes_by_id, self.tracks_by_frame)

        return pred_list

    def get_predictions(self, scene, track):
        pred_list = []
        
        scene_goals = [np.zeros((len(track), 2))]
        
        if len(scene)>0:
            # Get all predictions in parallel. Faster!
            pred_list = self.process_scene(self.predictor, self.model_name, track, scene_goals)

        return pred_list
    
    def process_scene(self, predictor, model_name, paths, scene_goal):
        # For each scene, get predictions
        predictions = self.predictor(paths, scene_goal, n_predict=self.pred_length, obs_length=self.obs_length, modes=self.modes)
        
        return predictions


if __name__ == '__main__':
    predictor = TrajNetPredictor()
    pred_list = predictor.prediction_function([[[0.1, 1.2, 1], [0.2, 1.3, 1]], [[1.2, 1.0, 2], [1.1, 0.9, 2]]])
    print(pred_list)

