import cv2
import numpy as np
import copy
from tools.utils import Utils
import os
import sys
import rospkg
rospack=rospkg.RosPack()
abs_path_to_perception=rospack.get_path('perception')
path_mmpose=os.path.abspath(os.path.join(abs_path_to_perception,"../../mmpose"))
print(f"path to mmpose {path_mmpose}")
sys.path.append(path_mmpose)
from mmpose.apis import (collect_multi_frames, extract_pose_sequence,
                         get_track_id, inference_pose_lifter_model,
                         inference_top_down_pose_model, init_pose_model,
                         process_mmdet_results, vis_3d_pose_result)
from mmpose.core import Smoother
from mmpose.datasets import DatasetInfo
from mmpose.models import PoseLifter, TopDown

class Keypoints3D():
    def __init__(self, device, img_resolution, show3D=False, save_video_keypoints=False,smooth=False):
        self.device=device
        self.resolution=img_resolution
        self.smooth= smooth
        self.frame_idx=0
        self.show_3Dkeypoints=show3D
        self.save_video_keypoints=save_video_keypoints
        self.name_video_keypoints="3DKeypoints.mp4"
        if save_video_keypoints:
            self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.fps = 2.5
            self.writer = None
        self.init_keypoints()
        self.init_3Dkeypoints()
        
    def init_keypoints(self):
        print("in init keypoints")
        pose_detector_config=os.path.join(path_mmpose,"configs/body/2d_kpt_sview_rgb_img/topdown_heatmap/coco/hrnet_w48_coco_256x192.py")
        pose_detector_checkpoint="https://download.openmmlab.com/mmpose/top_down/hrnet/hrnet_w48_coco_256x192-b9e0b3ab_20200708.pth"
        self.pose_det_model = init_pose_model(
            pose_detector_config,
            pose_detector_checkpoint,
            device=self.device)

        self.pose_det_dataset = self.pose_det_model.cfg.data['test']['type']

        self.dataset_info = self.pose_det_model.cfg.data['test'].get('dataset_info', None)
        if self.dataset_info is None:
            warnings.warn(
                'Please set `dataset_info` in the config.'
                'Check https://github.com/open-mmlab/mmpose/pull/663 for details.',
                DeprecationWarning)
        else:
            self.dataset_info = DatasetInfo(self.dataset_info)
            #l.319
        
        self.pose_det_results_list = []
        self.next_id = 0
        self.pose_det_results = []

    def inference_keypoints(self, frame, bbox):
        pose_det_results_last = self.pose_det_results
        bbox=Utils.bbox_xcentycentwh_to_xtlytlwh(bbox)
        self.pose_det_results, _ = inference_top_down_pose_model(
            self.pose_det_model,
            frame,
            [{"bbox":bbox}],
            bbox_thr=None,
            format='xywh',
            dataset=self.pose_det_dataset,
            dataset_info=self.dataset_info,
            return_heatmap=None,
            outputs=None)
        # get track id for each person instance
        self.pose_det_results, self.next_id = get_track_id(
            self.pose_det_results,
            pose_det_results_last,
            self.next_id,
            use_oks=False,
            tracking_thr=0.3)

        # convert keypoint definition
        for res in self.pose_det_results:
            keypoints = res['keypoints']
            res['keypoints'] = Utils.convert_keypoint_definition(
                keypoints, self.pose_det_dataset, self.pose_lift_dataset)
        
        self.pose_det_results_list.append(copy.deepcopy(self.pose_det_results))

    def init_3Dkeypoints(self):
        print("in init 3D keypoints")
        pose_lifter_config=os.path.join(path_mmpose,"configs/body/3d_kpt_sview_rgb_vid/video_pose_lift/h36m/videopose3d_h36m_243frames_fullconv_supervised_cpn_ft.py")
        pose_lifter_checkpoint="https://download.openmmlab.com/mmpose/body3d/videopose/videopose_h36m_243frames_fullconv_supervised_cpn_ft-88f5abbb_20210527.pth"
        self.pose_lift_model = init_pose_model(
        pose_lifter_config,
        pose_lifter_checkpoint,
        device=self.device)
        self.pose_lift_dataset = self.pose_lift_model.cfg.data['test']['type']

        # convert keypoint definition
        # for pose_det_results in self.pose_det_results_list:
        #     print("in keypoint conversion 1!!")
        #     print(self.pose_det_results_list)
        #     for res in pose_det_results:
        #         print("in keypoint conversion 2!!")
        #         keypoints = res['keypoints']
        #         print(f"keypoints before {keypoints}")
        #         res['keypoints'] = Utils.convert_keypoint_definition(
        #             keypoints, self.pose_det_dataset, self.pose_lift_dataset)
        #         print(f"keypoints after {res['keypoints']}")
        
        # load temporal padding config from model.data_cfg
        if hasattr(self.pose_lift_model.cfg, 'test_data_cfg'):
            self.data_cfg = self.pose_lift_model.cfg.test_data_cfg
        else:
            self.data_cfg = self.pose_lift_model.cfg.data_cfg

        # build pose smoother for temporal refinement
        if self.smooth:
            self.smoother = Smoother(
                filter_cfg=os.path.join(path_mmpose,'configs/_base_/filters/one_euro.py'),
                keypoint_key='keypoints',
                keypoint_dim=2)
        else:
            self.smoother = None

        self.num_instances = -1
        self.pose_lift_dataset_info = self.pose_lift_model.cfg.data['test'].get(
            'dataset_info', None)
        if self.pose_lift_dataset_info is None:
            warnings.warn(
                'Please set `dataset_info` in the config.'
                'Check https://github.com/open-mmlab/mmpose/pull/663 for details.',
                DeprecationWarning)
        else:
            self.pose_lift_dataset_info = DatasetInfo(self.pose_lift_dataset_info)

    def inference_3Dkeypoints(self, frame, bbox):
        self.inference_keypoints(frame,bbox)
        pose_results_2d = extract_pose_sequence(
            self.pose_det_results_list,
            frame_idx=self.frame_idx,
            causal=self.data_cfg.causal,
            seq_len=self.data_cfg.seq_len,
            step=self.data_cfg.seq_frame_interval)
        self.frame_idx+=1

        # smooth 2d results
        if self.smoother:
            pose_results_2d = self.smoother.smooth(pose_results_2d)

        # 2D-to-3D pose lifting
        pose_lift_results = inference_pose_lifter_model(
            self.pose_lift_model,
            pose_results_2d=pose_results_2d,
            dataset=self.pose_lift_dataset,
            dataset_info=self.pose_lift_dataset_info,
            with_track_id=True,
            image_size=self.resolution,
            norm_pose_2d=False)

        # Pose processing
        pose_lift_results_vis = []
        for idx, res in enumerate(pose_lift_results):
            keypoints_3d = res['keypoints_3d']
            # exchange y,z-axis, and then reverse the direction of x,z-axis
            keypoints_3d = keypoints_3d[..., [0, 2, 1]]
            keypoints_3d[..., 0] = -keypoints_3d[..., 0]
            keypoints_3d[..., 2] = -keypoints_3d[..., 2]
            # rebase height (z-axis)
            rebase_keypoint_height=True
            if rebase_keypoint_height:
                keypoints_3d[..., 2] -= np.min(
                    keypoints_3d[..., 2], axis=-1, keepdims=True)
            res['keypoints_3d'] = keypoints_3d
            # add title
            det_res = self.pose_det_results[idx]
            instance_id = det_res['track_id']
            res['title'] = f'Prediction ({instance_id})'
            # only visualize the target frame
            res['keypoints'] = det_res['keypoints']
            res['bbox'] = det_res['bbox']
            res['track_id'] = instance_id
            pose_lift_results_vis.append(res)

        # Visualization
        if self.num_instances < 0:
            self.num_instances = len(pose_lift_results_vis)
        img_vis = vis_3d_pose_result(
            self.pose_lift_model,
            result=pose_lift_results_vis,
            img=frame,
            dataset=self.pose_lift_dataset,
            dataset_info=self.pose_lift_dataset_info,
            out_file=None,
            radius=3,
            thickness=1,
            num_instances=self.num_instances,
            show=False)

        if self.show_3Dkeypoints:
            cv2.imshow('Keypoints',img_vis)
            cv2.waitKey(1)
        if self.save_video_keypoints:
            if self.writer is None:
                self.writer = cv2.VideoWriter(
                    os.path.join(abs_path_to_perception,self.name_video_keypoints), self.fourcc,
                    self.fps, (img_vis.shape[1], img_vis.shape[0]))
            self.writer.write(img_vis)
        
        return res
