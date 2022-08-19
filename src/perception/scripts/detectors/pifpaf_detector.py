#!/usr/bin/env python3
# VITA, EPFL
import io
import numpy as np
import openpifpaf
import PIL
import requests
import torch
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import math


class Detector_pifpaf():
    def __init__(self, detector_size, verbose):

        self.bboxes_hip_prev = [[0.0, 0.0], [0.0, 0.0]]

        self.device = torch.device('cuda')

        net_cpu, _ = openpifpaf.network.factory(checkpoint='shufflenetv2k16w', download_progress=False)
        self.net = net_cpu.to(self.device)

        openpifpaf.decoder.CifSeeds.threshold = 0.5
        openpifpaf.decoder.nms.Keypoints.keypoint_threshold = 0.2
        openpifpaf.decoder.nms.Keypoints.instance_threshold = 0.2
        self.processor = openpifpaf.decoder.factory_decode(self.net.head_nets, basenet_stride=self.net.base_net.stride)

        self.preprocess = openpifpaf.transforms.Compose([
        openpifpaf.transforms.NormalizeAnnotations(),
        openpifpaf.transforms.CenterPadTight(16),
        openpifpaf.transforms.EVAL_TRANSFORM,
        ])

    # Find the patient in the frame
    def bboxes_closest(self, bboxes_legs):

        bboxes_hip = []
        d_list = []

        for i, e in enumerate(bboxes_legs):
            bboxes_hip.append([e[0], e[1]])
            d_list.append(self.calculate_distance(bboxes_hip[i]))

        idx = d_list.index(min(d_list))

        return bboxes_hip[idx], idx 
    
    # Hip Distance between current and past detection
    def calculate_distance(self, bboxes_hip):

        d = math.sqrt((bboxes_hip[0][0]-self.bboxes_hip_prev[0][0])**2 + (bboxes_hip[0][1]-self.bboxes_hip_prev[0][1])**2) + math.sqrt((bboxes_hip[1][0]-self.bboxes_hip_prev[1][0])**2 + (bboxes_hip[1][1]-self.bboxes_hip_prev[1][1])**2)

        return d

    # Openpifpaf algorithm
    def forward(self, image, verbose):
        
        # Openpifpaf preprocess
        data = openpifpaf.datasets.PilImageList([image], preprocess=self.preprocess)
        loader = torch.utils.data.DataLoader(
        data, batch_size=1, pin_memory=False, 
        collate_fn=openpifpaf.datasets.collate_images_anns_meta)

        # Openpifpaf process: keypoints
        for images_batch, _, __ in loader:
            predictions = self.processor.batch(self.net, images_batch, device=self.device)[0]

        bbox = []
        label = []
        key = ['left_hip', 'right_hip', 'left_knee', 'right_knee', 'left_ankle', 'right_ankle']
        bboxes_legs = []

        # For every person in the image
        for pred in predictions:
            x_list = []
            y_list = []
            x_leg_list = [2.0]*len(key)
            y_leg_list = [2.0]*len(key)
            bboxes_legs_pixels = []

            for idx, e in enumerate(pred.data):
                if e[0]!=0.0:
                    x_list.append(e[0])
                    y_list.append(e[1])
            
                if pred.keypoints[idx] in key:
                    x_leg_list[key.index(pred.keypoints[idx])] = e[0]
                    y_leg_list[key.index(pred.keypoints[idx])] = e[1]

            # Bounding Box limits
            x_min = int(min(x_list))#/np.sqrt(downscale))
            y_min = int(min(y_list))#/np.sqrt(downscale))
            x_max = int(max(x_list))#/np.sqrt(downscale))
            y_max = int(max(y_list))#/np.sqrt(downscale))

            bbox.append([x_min, y_min, x_max-x_min, y_max-y_min])
            label.append([1])

            # (x,y) of leg Keypoints
            for i in range(len(x_leg_list)):
                bboxes_legs_pixels.append([x_leg_list[i], y_leg_list[i]])

            bboxes_legs.append(bboxes_legs_pixels)

        # If there is no detection, return 0s
        if len(bbox) == 0:
            bbox = [[0.0, 0.0, 0.0, 0.0]]
            label = [[0]]
            bboxes_legs = [[0.0, 0.0, 0.0, 0.0]]*6
            bboxes_legs_pixels = [[0.0, 0.0, 0.0, 0.0]]*6

        # If there is detection, select the person who was closest to the last seen one
        else:
            bboxes_hip_act, idx = self.bboxes_closest(bboxes_legs)
            self.bboxes_hip_prev = bboxes_hip_act
            bboxes_legs = bboxes_legs[idx]
            bbox = [bbox[idx]]

        return bbox, label, bboxes_legs
