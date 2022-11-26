import numpy as np
import torch
from mmtrack.apis import inference_mot, init_model
import os
from tools.utils import Utils
import rospkg
#from trackers.base_mmtracking import BaseTracker


class MotTracker():

    def __init__(self, tracking_conf="0.5", device='gpu', verbose=0):
        '''
        init_model parameters: 
        tracker_name (Stark/Siamese) 
        desired device to specify cpu if wanted
        '''
        #FIX: download weights and set the path to chckpt and cfg
        desired_device = device

        #use rospackage to be able to find path for ROS packages
        #print(rospack.list())
        rospack=rospkg.RosPack()
        abs_path_to_perception=rospack.get_path('perception')

        path_config=os.path.abspath(os.path.join(abs_path_to_perception,"../../mmtracking/configs/mot/bytetrack/bytetrack_yolox_x_crowdhuman_mot17-private.py"))#"configs/stark_st2_r50_50e_lasot.py"
        path_model=os.path.abspath(os.path.join(abs_path_to_perception,"scripts/trackers/weights/bytetrack_yolox_x_crowdhuman_mot17-private-half_20211218_205500-1985c9f0.pth"))#"weights/stark_st2_r50_50e_lasot_20220416_170201-b1484149.pth"

        cpu = 'cpu' == desired_device
        cuda = not cpu and torch.cuda.is_available()
        self.device = torch.device('cuda:0' if cuda else 'cpu')
        self.tracker = init_model(path_config, path_model, self.device) 
        #prog_bar = mmcv.ProgressBar(len(imgs))
        self.conf_thresh=tracking_conf
        self.frame=0
        self.verbose=verbose
        if self.verbose:
            print(f"-> Using tracker {tracker_name} from MMTracking")


    def forward(self, img: np.ndarray) -> list:
        '''
        cut_imgs: img parts cut from img at bbox positions
        detections: bboxes from YOLO detector
        img: original image
        -> bbox
        '''
        bbox_list=[]
        result = inference_mot(self.tracker, img, frame_id=self.frame)
        self.frame+=1
        track_bboxes=result['track_bboxes']
        track_bboxes=np.squeeze(track_bboxes)
        if np.array(track_bboxes).ndim==1:
            track_bboxes=[track_bboxes]
        for detection in track_bboxes:
            id=detection[0]
            bbox=detection[1:5]
            conf=detection[5]
            bbox=Utils.bbox_x1y1x2y2_to_xcentycentwh(bbox)
            bbox = [int(x) for x in bbox]
            if self.verbose >=3: 
                print(f"id {id} with bbox{bbox}")
            bbox_list.append(bbox)
        if bbox_list:
            return bbox_list
        else:
            return None