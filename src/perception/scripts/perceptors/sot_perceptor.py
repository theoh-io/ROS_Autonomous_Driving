import numpy as np
import os 
# import logging
import time
import cv2
from tools.utils import Utils
from perceptors.base_perceptor import BasePerceptor




class SotPerceptor(BasePerceptor):
    def forward(self, img):
        image=self.preproc(img)
        # Detection
        tic1 = time.perf_counter()
        bbox_list = self.detector.forward(image)
        toc1 = time.perf_counter()
        if self.verbose_level >=2:
            print(f"Elapsed time for detector forward pass: {(toc1 - tic1) * 1e3:.1f}ms")

        # #Solve this to make it clearer always bbox_list = None if no detections
        # if bbox_list is not None:
        #     # if self.use_img_transform:
        #     cut_imgs = Utils.crop_img_bbox(bbox_list,image)
        #     # if self.verbose and cut_imgs is not None: print("in preprocessing: ", bbox_list)
        #     # else:
        #     #     cut_imgs = None
        # else:
        #     if self.verbose is True: print("No person detected.")
        cut_imgs = None

        # Tracking
        tic2 = time.perf_counter()
        if bbox_list is not None:
            bbox = self.tracker.forward(cut_imgs,bbox_list,image)
            toc2 = time.perf_counter()
            if self.verbose_level>= 2:
                print(f"Elapsed time for tracker forward pass: {(toc2 - tic2) * 1e3:.1f}ms")
        else: 
            bbox=None

        if self.keypoints_activ and bbox:
            tic3 = time.perf_counter()
            results_keypoints=self.Keypoints3D.inference_3Dkeypoints(image, bbox)
            toc3 = time.perf_counter()
            if self.verbose_level >=2:
                print(f"Elapsed time for 3D keypoints forward pass: {(toc3 - tic3) * 1e3:.1f}ms")

        else:
            results_keypoints=None
        
        if self.show:
            Utils.bbox_vis(bbox, image)
        toc4 = time.perf_counter()
        if self.verbose_level >=2:
                print(f"Elapsed time for perceptor forward pass: {(toc4 - tic1) * 1e3:.1f}ms")

        return bbox, None, results_keypoints, image
