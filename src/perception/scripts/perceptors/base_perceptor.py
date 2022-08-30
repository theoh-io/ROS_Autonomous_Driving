from detectors.yolov5_detector import Yolov5Detector
from trackers.mmtracking_sot import SotaTracker
from keypoints.keypoints3d import Keypoints3D
from PIL import Image
import cv2
import numpy as np
import copy
import torch
from tools.utils import Utils
#add the path to mmpose
import os
import sys
#import rospkg
# rospack=rospkg.RosPack()
# abs_path_to_perception=rospack.get_path('perception')
# path_mmpose=os.path.abspath(os.path.join(abs_path_to_perception,"../../mmpose"))
# print(f"path to mmpose {path_mmpose}")
# sys.path.append(path_mmpose)

from mmpose.apis import (collect_multi_frames, extract_pose_sequence,
                         get_track_id, inference_pose_lifter_model,
                         inference_top_down_pose_model, init_pose_model,
                         process_mmdet_results, vis_3d_pose_result)
from mmpose.core import Smoother
from mmpose.datasets import DatasetInfo
from mmpose.models import PoseLifter, TopDown



class BasePerceptor():

    def __init__(self, width=640, height=480, channels=3, downscale=4, 
                    detector=Yolov5Detector(), detector_size="default",
                    tracker=None, tracker_model=None, tracking_conf=0.5,
                    type_input="opencv", keypoints=False, save_video_keypoints=False,
                    device="gpu", show="True", show3D="False", verbose=0):

        cpu = 'cpu' == device
        cuda = not cpu and torch.cuda.is_available()
        self.device = torch.device('cuda:0' if cuda else 'cpu')

        # perceptor expected input image dimensions
        self.width = int(width/downscale)
        self.height = int(height/downscale)
        self.resolution=(width, height)
        self.downscale = downscale
        self.verbose_level=verbose

        # Image received size data.
        self.data_size = int(self.width * self.height * channels)

        self.detector=detector(detector_size, verbose=self.verbose_level)
        if tracker:
            self.tracker=tracker(tracker_model, tracking_conf, verbose=self.verbose_level)
        self.type_input=type_input
        self.show=show

        #3D Pose Estimation
        self.keypoints_activ=keypoints
        if self.keypoints_activ:
            self.Keypoints3D=Keypoints3D(self.device, self.resolution, show3D, save_video_keypoints)

        if self.verbose_level:
            print("Initializing Perceptor")
            print(f"-> Input image of type {self.type_input} and downscale {self.downscale}")

    def preproc(self,image):
        # Adapt image to detector requirements
        pil_image = Image.frombytes('RGB', (self.width,self.height), image)
        opencvImage = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
        opencvImage = cv2.cvtColor(opencvImage,cv2.COLOR_BGR2RGB)
        self.opencvImage=opencvImage

        if self.type_input == "opencv":
            image = opencvImage
        
        elif self.type_input == "pil":
            image = pil_image

        return image

    def forward(self, image):
        raise NotImplementedError("perceptor Base Class does not provide a forward method.")


