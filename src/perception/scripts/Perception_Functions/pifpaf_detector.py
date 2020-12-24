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


class Detector_pifpaf():
    def __init__(self):

        self.device = torch.device('cpu')

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

    
    def forward(self, image, downscale):

        start = time.time()
        data = openpifpaf.datasets.PilImageList([image], preprocess=self.preprocess)

        loader = torch.utils.data.DataLoader(
        data, batch_size=1, pin_memory=False, 
        collate_fn=openpifpaf.datasets.collate_images_anns_meta)

        for images_batch, _, __ in loader:
            predictions = self.processor.batch(self.net, images_batch, device=self.device)[0]

        bbox = []
        label = []

        for pred in predictions:
            x_list = []
            y_list = []

            for e in pred.data:
                if e[0]!=0.0:
                    x_list.append(e[0])
                    y_list.append(e[1])
            
            x_min = int(min(x_list)/np.sqrt(downscale))
            y_min = int(min(y_list)/np.sqrt(downscale))
            x_max = int(max(x_list)/np.sqrt(downscale))
            y_max = int(max(y_list)/np.sqrt(downscale))

            bbox.append([x_min, y_min, x_max-x_min, y_max-y_min])
            label.append([1])

        if len(bbox) == 0:
            bbox = [[0.0, 0.0, 0.0, 0.0]]
            label = [[0]]

        return bbox, label