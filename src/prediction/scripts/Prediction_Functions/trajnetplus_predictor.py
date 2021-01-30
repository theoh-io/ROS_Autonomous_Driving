#!/usr/bin/env python3
# VITA, EPFL
import os
from collections import OrderedDict, defaultdict
import argparse
import pickle
import time

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

    def __init__(self, dt=0.35, pred_horizon=2.0, obs_length=3, model_name="LSTM", model="", tag=2):
        self.unimodal = True
        self.modes = 3
        self.model_name = model_name
        self.obs_length = obs_length
        self.pred_length = int(pred_horizon/dt)
        self.dt = dt
               
        if 'SGAN' in self.model_name:
            self.predictor = trajnetbaselines.sgan.SGANPredictor.load(model)
            self.goal_flag = self.predictor.model.generator.goal_flag

        else:
            self.predictor = trajnetbaselines.lstm.LSTMPredictor.load(model)
            self.goal_flag = self.predictor.model.goal_flag

        device = torch.device('cpu')
        self.predictor.model.to(device)
        
    # position_present_past: present and past detections --> [[[x11,y11,1],[x12,y12,1],[x13,y13,1]],[[x21,y21,2],[x22,y22,2],[x23,y23,2]]] (if n_past=3)
    def prediction_function(self, position_present_past):
        start = time.time()

        if len(position_present_past)>0:

            for i in range(len(position_present_past)):

                for j in range(len(position_present_past[i])):
                    position_present_past[i][j] = position_present_past[i][j][0:2]

            position_present_past = np.array(position_present_past)
            position_present_past = np.transpose(position_present_past, (1, 0, 2))
            pred_list = self.get_predictions(position_present_past)

            return pred_list

        return []

    def get_predictions(self, xy):
        pred_list = []
        scene_goal = np.zeros((np.shape(xy)[1], np.shape(xy)[2]))
        pred_list = self.process_scene(self.predictor, self.model_name, xy, scene_goal)

        return pred_list
    
    def process_scene(self, predictor, model_name, xy, scene_goal):
        # For each scene, get predictions
        pred_list = []
        new_args = Argument()
        predictions = self.predictor(xy, scene_goal, n_predict=self.pred_length, obs_length=self.obs_length, modes=self.modes, args=new_args)
        
        for pred in predictions[0]:
            if len(pred[0])>0:
                pred_list.append(pred.tolist())

        for idx, element in enumerate(pred_list):

            for i in range(len(element)):

                if idx == 0:
                    element[i].append((i+1)*self.dt)

                else:
                    element[i][0].append((i+1)*self.dt)
                    element[i] = element[i][0]

        return pred_list


class Argument:
    def __init__(self):
        self.normalize_scene = False


if __name__ == '__main__':
    predictor = TrajNetPredictor()
    pred_list = predictor.prediction_function([[[0.1, 1.2], [0.2, 1.3],[0.3, 1.4]],[[0.6, 1.9], [0.6, 2.3],[0.7, 2.4]]])
    print(pred_list)
