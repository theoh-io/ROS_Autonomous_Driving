import torch
import numpy as np
from mmtrack.apis import inference_sot, init_model
import os
import rospkg

#from perceptionloomo.utils.utils import Utils


class SotaTracker():
    def __init__(self, tracker_name="Stark", tracking_conf="0.5", device='gpu', verbose="False"):
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

        if tracker_name=="Stark" or tracker_name=="stark":
            
            path_config=os.path.abspath(os.path.join(abs_path_to_perception,"../../mmtracking/configs/sot/stark/stark_st2_r50_50e_lasot.py"))#"configs/stark_st2_r50_50e_lasot.py"
            path_model=os.path.abspath(os.path.join(abs_path_to_perception,"scripts/trackers/weights/stark_st2_r50_50e_lasot_20220416_170201-b1484149.pth"))#"weights/stark_st2_r50_50e_lasot_20220416_170201-b1484149.pth"
        elif tracker_name=="Siamese" or tracker_name=="siamese" or tracker_name=="siam" or tracker_name=="Siam":
            print("Siamese is still not implemented")


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


    def forward(self, cut_imgs: list, detections: list, img: np.ndarray) -> list:
        '''
        cut_imgs: img parts cut from img at bbox positions
        detections: bboxes from YOLO detector
        img: original image
        -> bbox
        '''
        if self.frame==0:
            init_bbox=detections[0]
            self.new_bbox=Utils.bbox_xcentycentwh_to_x1y1x2y2(init_bbox)

        #input of the bbox format is x1, y1, x2, y2
        result = inference_sot(self.tracker, img, self.new_bbox, frame_id=self.frame)
        
        self.frame+=1
        track_bbox=result['track_bboxes']
        #remove last index -1
        confidence=track_bbox[4]
        if self.verbose:
            print(f"Tracking conf is: {confidence}")
        bbox=track_bbox[:4]#[test_bbox[0], test_bbox[1], test_bbox[2]-test_bbox[0], test_bbox[3]-test_bbox[1]]
        
        if confidence>self.conf_thresh:
            #changing back format from (x1, y1, x2, y2) to (xcenter, ycenter, width, height) before writing
            bbox=Utils.bbox_x1y1x2y2_to_xcentycentwh(bbox)
            bbox = [int(x) for x in bbox]
        else:
            if self.verbose:
                print("Under Tracking threshold")
            #bbox=[0, 0, 0, 0]
            bbox=None

        return bbox

