#!/usr/bin/env python3
# VITA, EPFL
import os
from collections import OrderedDict
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

class TrajNetPredictor():

    def __init__(self, dt=1.0, pred_horizon=1.0, obs_length=1, model_name="SGAN", model="orca_opt", tag=2):
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
            scene["scene " + str(self.frame_number) + ": " + str(idx)] = {"id": self.frame_number, "p": detection[0][2], "s": self.frame_number, "e": self.frame_number+self.obs_length, "fps": self.fps, "tag": self.tag}
            
            for i in range(len(detection)):
                track["track " + str(self.frame_number + i) + ": " + str(idx)  + " - " + str(i)] = {"f": self.frame_number + i, "p": detection[i][2], "x": detection[i][0], "y": detection[i][1], "pred_number": min(3, len(position_present_past))}
        
        pred_list = self.get_predictions(scene, track)

        return pred_list

    def get_predictions(self, scene, track):
        scene = tqdm(scene)
        scene_goals = track
        ## Get all predictions in parallel. Faster!
        pred_list = Parallel(n_jobs=5)(delayed(self.process_scene)(self.predictor, self.model_name, paths, scene_goal)
                                        for (_, _, paths), scene_goal in zip(scene, scene_goals))

        return pred_list
    
    def process_scene(self, predictor, model_name, paths, scene_goal):
        ## For each scene, get predictions
        predictions = self.predictor(paths, scene_goal, n_predict=self.pred_length, obs_length=self.obs_length, modes=self.modes)
        return predictions


if __name__ == '__main__':
    predictor = TrajNetPredictor()
    pred_list = predictor.prediction_function([[[0.1, 1.2, 1], [0.2, 1.3, 1]], [[1.2, 1.0, 2], [1.1, 0.9, 2]]])
    prtint(pred_list)

