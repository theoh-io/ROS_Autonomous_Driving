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
        bbox_det = self.detector.forward(image)
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


        

        # Single Object Tracking (Target)
        tic2 = time.perf_counter()
        if bbox_det is not None:
            bbox_sot = self.tracker.forward(cut_imgs,bbox_det,image)
            toc2 = time.perf_counter()
            if self.verbose_level>= 3:
                print(f"bbox from SOT {bbox_sot}")
            if self.verbose_level>= 2:
                print(f"Elapsed time for tracker forward pass: {(toc2 - tic2) * 1e3:.1f}ms")
        else: 
            bbox_sot=None

        #Multi-Object Tracking
        bbox_final=None
        if self.mot_activated:
            tic3 = time.perf_counter()
            if bbox_det is not None:
                if bbox_sot:
                    bbox_final=[bbox_sot]
                else:
                    bbox_final=[[0, 0, 0, 0]]
                mot_bbox=self.mot_tracker.forward(image)
                if self.verbose_level>= 3:
                    print(f"bbox candidates from MOT {mot_bbox}")
                toc3 = time.perf_counter()
                if self.verbose_level>= 2:
                    print(f"Elapsed time for MOT tracker forward pass: {(toc3 - tic3) * 1e3:.1f}ms")
                #find correspondence with SOT bbox
                found_correspondence=0
                if mot_bbox:
                    #if len(mot_bbox)>1:
                    for i, mot_indiv in enumerate(mot_bbox):
                        if bbox_sot:
                            if self.verbose_level>= 3:
                                print(f" debug dist mot: {Utils.calculate_distance(mot_indiv, bbox_sot)}")
                            #IMPROVEMENT Better identification of SOT BBOX
                            if Utils.calculate_distance(mot_indiv, bbox_sot) < 10:
                                if self.verbose_level>= 3:
                                    print(f"identified bbox from sot :{bbox_sot}, {mot_indiv}")
                                found_correspondence+=1
                            else:
                                bbox_final.append(mot_indiv)
                            #mot_bbox.pop(i)
                            #handle no correspondance: false SOT 
                        else:
                            bbox_final.append(mot_indiv)

                    if found_correspondence==0:
                        #wrong SOT detection: reinitialize
                        bbox_sot=None
                        bbox_final=[[0, 0, 0, 0]] #firstbbox empty to show that target is not here
                        for i, mot_indiv in enumerate(mot_bbox):
                            if i==0:
                                self.tracker.new_bbox=Utils.bbox_xcentycentwh_to_x1y1x2y2(mot_indiv)
                            bbox_final.append(mot_indiv)
                    elif found_correspondence>1:
                        print(f"problem multiple correspondences")

                elif bbox_sot is None:
                    #handle the case both no detections
                    bbox_final=None
                    # elif len(mot_bbox)==1 and bbox_sot:
                    #     #only 1 detection should be the same as SOT
                    #     if Utils.calculate_distance(mot_bbox[0], bbox_sot) < 5:
                    #                 print(f"identified bbox from sot :{bbox_sot}, {mot_bbox[0]}")
                    #     else:
                    #         #wrong SOT detection: reinitialize
                    #         bbox_sot=None
                    #         bbox_final.append([0, 0, 0, 0]) #firstbbox empty to show that target is not here
                    #         bbox_final.append(mot_bbox[0])
                        
                        

        elif bbox_sot:
            bbox_final=[bbox_sot]
        
        if self.show:
            #plot the rectangle on a copy of the image to be able to transmit to pose_est without rectangle
            image_bbox=image.copy()
            if self.mot_activated:
                Utils.mult_bbox_vis(bbox_final, image_bbox)
            else:
                Utils.bbox_vis(bbox_sot, image_bbox)
        toc4 = time.perf_counter()
        if self.verbose_level >=2:
                print(f"Elapsed time for perceptor forward pass: {(toc4 - tic1) * 1e3:.1f}ms")

        return bbox_final, image
