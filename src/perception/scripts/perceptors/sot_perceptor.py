# import numpy as np
# import os 
# import logging
# import time
# from typing import List
# # from perceptionloomo.detectors.pose_detectors import PoseDetector
# from perceptionloomo.detectors.pose_yolo_detector import PoseYoloDetector
# from perceptionloomo.trackers.fused_ds_reid import FusedDsReid
# from perceptionloomo.utils.utils import Utils
# from perceptionloomo.deep_sort.utils.parser import YamlParser


class SotPerceptor(base_perceptor.BasePerceptor):

    def forward(self, image):

        # Detection
        tic1 = time.perf_counter()
        bbox_list = self.detector.forward(image)
        toc1 = time.perf_counter()
        if self.verbose:
            print(f"Elapsed time for detector forward pass: {(toc1 - tic1) * 1e3:.1f}ms")

        #Solve this to make it clearer always bbox_list = None if no detections
        if bbox_list is not None:
            # if self.use_img_transform:
            cut_imgs = Utils.crop_img_parts_from_bboxes(bbox_list,img,self.img_processing)
            # if self.verbose and cut_imgs is not None: print("in preprocessing: ", bbox_list)
            # else:
            #     cut_imgs = None
        else:
            if self.verbose is True: print("No person detected.")
            bbox = None

        # Tracking
        tic2 = time.perf_counter()
        if bbox_list is not None:
            bbox = self.tracker.forward(cut_imgs,bbox_list,img)
            toc2 = time.perf_counter()
            if self.verbose:
                print(f"Elapsed time for tracker forward pass: {(toc2 - tic2) * 1e3:.1f}ms")

        toc3 = time.perf_counter()
        if self.verbose:
                print(f"Elapsed time for perceptor forward pass: {(toc3 - tic1) * 1e3:.1f}ms")

        return bbox
