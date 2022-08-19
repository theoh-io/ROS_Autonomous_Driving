import numpy as np
import torch
import cv2
from PIL import Image

#FrameGraber
#Bbox Formatting

#bbox cropping

class Utils():
    @staticmethod
    #FIX: use BBOX format to crop, output format tensor image ?
    def crop_img_bbox(bbox_list: list, img: np.ndarray):
        img_list=[]
        print(f"bbox_list: {bbox_list}")
        print(type(img))
        if bbox_list is not None:
            for i in range(bbox_list.shape[0]):
                bbox_indiv=bbox_list[i]
                print(f"bbox_indiv{bbox_indiv}")
                crop_img=img[int((bbox_indiv[1]-bbox_indiv[3]/2)):int((bbox_indiv[1]+bbox_indiv[3]/2)), int((bbox_indiv[0]-bbox_indiv[2]/2)):int((bbox_indiv[0]+bbox_indiv[2]/2))]
                #to apply the normalization need a PIL image
                # PIL RGB while CV is BGR.
                #crop_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB)
                #crop_img = Image.fromarray(crop_img)
                #tensor_img_indiv=image_processing(crop_img)

                tensor_img_indiv=torch.unsqueeze(torch.from_numpy(crop_img), 0)
                img_list.append(tensor_img_indiv)
            #print()
            np_img_list=np.asarray(img_list)
            print(img_list.shape)
            tensor_img=torch.from_numpy(np_img_list)
            #tensor_img=torch.cat(img_list)
            return tensor_img
        else:
            return None

    @staticmethod
    def bbox_xcentycentwh_to_x1y1x2y2(bbox):
        #convert from (xcenter,y_center, width, height) to (x1,y1,x2,y2)
        offset_x=int(bbox[2]/2)
        offset_y=int(bbox[3]/2)
        new_bbox=[0, 0, 0, 0]
        new_bbox[0]=bbox[0]-offset_x
        new_bbox[1]=bbox[1]-offset_y
        new_bbox[2]=bbox[0]+offset_x
        new_bbox[3]=bbox[1]+offset_y
        return new_bbox

    @staticmethod
    def bbox_x1y1x2y2_to_xcentycentwh(bbox):
        #convert from (x1,y1,x2,y2) to (xcenter,y_center, width, height)
        offset_x=int((bbox[2]-bbox[0])/2)
        offset_y=int((bbox[3]-bbox[1])/2)
        bbox[0]=bbox[0]+offset_x
        bbox[1]=bbox[1]+offset_y
        bbox[2]=offset_x*2
        bbox[3]=offset_y*2
        return bbox

    @staticmethod
    def bbox_xcentycentwh_to_xtlytlwh(bbox):
        #convert from (xcenter,ycenter,width,height) to (x_topleft,y_topleft, width, height)
        x_tl= bbox[0] - bbox[2]/2
        y_tl= bbox[1] - bbox[3]/2
        w= bbox[2]
        h= bbox[3]
        new_bbox=[x_tl, y_tl, w, h]
        return new_bbox

    @staticmethod
    def bbox_xtlytlwh_to_xcentycentwh(bbox):
        #convert from (x_topleft,y_topleft, width, height) to (xcenter,ycenter,width,height)
        x= bbox[0] + bbox[2]/2
        y= bbox[1] + bbox[3]/2
        w= bbox[2]
        h= bbox[3]
        new_bbox=[x, y, w, h]
        return new_bbox


    @staticmethod
    def bbox_scaling(bbox, scale):
        #scaling using the parameter on width and height
        bbox[2]=scale*bbox[2]
        bbox[3]=scale*bbox[3]
        return bbox

    @staticmethod
    def bbox_vis(bbox, image):
        #input format (xcenter,ycenter,width,height) => cv2 (x_tl,y_tl,x_br,y_br)
        if bbox is not None:
            tl=(int(bbox[0]-bbox[2]/2), int(bbox[1]+bbox[3]/2)) 
            br= (int(bbox[0]+bbox[2]/2), int(bbox[1]-bbox[3]/2))
            cv2.rectangle(image, tl, br, (255,0,0), 2)
        cv2.imshow('Camera Loomo',image)
        cv2.waitKey(1)

    @staticmethod
    def convert_keypoint_definition(keypoints, pose_det_dataset,
                                pose_lift_dataset):
        """Convert pose det dataset keypoints definition to pose lifter dataset
        keypoints definition, so that they are compatible with the definitions
        required for 3D pose lifting.
        Args:
            keypoints (ndarray[K, 2 or 3]): 2D keypoints to be transformed.
            pose_det_dataset, (str): Name of the dataset for 2D pose detector.
            pose_lift_dataset (str): Name of the dataset for pose lifter model.
        Returns:
            ndarray[K, 2 or 3]: the transformed 2D keypoints.
        """
        assert pose_lift_dataset in [
            'Body3DH36MDataset', 'Body3DMpiInf3dhpDataset'
            ], '`pose_lift_dataset` should be `Body3DH36MDataset` ' \
            f'or `Body3DMpiInf3dhpDataset`, but got {pose_lift_dataset}.'

        coco_style_datasets = [
            'TopDownCocoDataset', 'TopDownPoseTrack18Dataset',
            'TopDownPoseTrack18VideoDataset'
        ]
        keypoints_new = np.zeros((17, keypoints.shape[1]), dtype=keypoints.dtype)
        if pose_lift_dataset == 'Body3DH36MDataset':
            if pose_det_dataset in ['TopDownH36MDataset']:
                keypoints_new = keypoints
            elif pose_det_dataset in coco_style_datasets:
                # pelvis (root) is in the middle of l_hip and r_hip
                keypoints_new[0] = (keypoints[11] + keypoints[12]) / 2
                # thorax is in the middle of l_shoulder and r_shoulder
                keypoints_new[8] = (keypoints[5] + keypoints[6]) / 2
                # spine is in the middle of thorax and pelvis
                keypoints_new[7] = (keypoints_new[0] + keypoints_new[8]) / 2
                # in COCO, head is in the middle of l_eye and r_eye
                # in PoseTrack18, head is in the middle of head_bottom and head_top
                keypoints_new[10] = (keypoints[1] + keypoints[2]) / 2
                # rearrange other keypoints
                keypoints_new[[1, 2, 3, 4, 5, 6, 9, 11, 12, 13, 14, 15, 16]] = \
                    keypoints[[12, 14, 16, 11, 13, 15, 0, 5, 7, 9, 6, 8, 10]]
            elif pose_det_dataset in ['TopDownAicDataset']:
                # pelvis (root) is in the middle of l_hip and r_hip
                keypoints_new[0] = (keypoints[9] + keypoints[6]) / 2
                # thorax is in the middle of l_shoulder and r_shoulder
                keypoints_new[8] = (keypoints[3] + keypoints[0]) / 2
                # spine is in the middle of thorax and pelvis
                keypoints_new[7] = (keypoints_new[0] + keypoints_new[8]) / 2
                # neck base (top end of neck) is 1/4 the way from
                # neck (bottom end of neck) to head top
                keypoints_new[9] = (3 * keypoints[13] + keypoints[12]) / 4
                # head (spherical centre of head) is 7/12 the way from
                # neck (bottom end of neck) to head top
                keypoints_new[10] = (5 * keypoints[13] + 7 * keypoints[12]) / 12

                keypoints_new[[1, 2, 3, 4, 5, 6, 11, 12, 13, 14, 15, 16]] = \
                    keypoints[[6, 7, 8, 9, 10, 11, 3, 4, 5, 0, 1, 2]]
            elif pose_det_dataset in ['TopDownCrowdPoseDataset']:
                # pelvis (root) is in the middle of l_hip and r_hip
                keypoints_new[0] = (keypoints[6] + keypoints[7]) / 2
                # thorax is in the middle of l_shoulder and r_shoulder
                keypoints_new[8] = (keypoints[0] + keypoints[1]) / 2
                # spine is in the middle of thorax and pelvis
                keypoints_new[7] = (keypoints_new[0] + keypoints_new[8]) / 2
                # neck base (top end of neck) is 1/4 the way from
                # neck (bottom end of neck) to head top
                keypoints_new[9] = (3 * keypoints[13] + keypoints[12]) / 4
                # head (spherical centre of head) is 7/12 the way from
                # neck (bottom end of neck) to head top
                keypoints_new[10] = (5 * keypoints[13] + 7 * keypoints[12]) / 12

                keypoints_new[[1, 2, 3, 4, 5, 6, 11, 12, 13, 14, 15, 16]] = \
                    keypoints[[7, 9, 11, 6, 8, 10, 0, 2, 4, 1, 3, 5]]
            else:
                raise NotImplementedError(
                    f'unsupported conversion between {pose_lift_dataset} and '
                    f'{pose_det_dataset}')

        elif pose_lift_dataset == 'Body3DMpiInf3dhpDataset':
            if pose_det_dataset in coco_style_datasets:
                # pelvis (root) is in the middle of l_hip and r_hip
                keypoints_new[14] = (keypoints[11] + keypoints[12]) / 2
                # neck (bottom end of neck) is in the middle of
                # l_shoulder and r_shoulder
                keypoints_new[1] = (keypoints[5] + keypoints[6]) / 2
                # spine (centre of torso) is in the middle of neck and root
                keypoints_new[15] = (keypoints_new[1] + keypoints_new[14]) / 2

                # in COCO, head is in the middle of l_eye and r_eye
                # in PoseTrack18, head is in the middle of head_bottom and head_top
                keypoints_new[16] = (keypoints[1] + keypoints[2]) / 2

                if 'PoseTrack18' in pose_det_dataset:
                    keypoints_new[0] = keypoints[1]
                    # don't extrapolate the head top confidence score
                    keypoints_new[16, 2] = keypoints_new[0, 2]
                else:
                    # head top is extrapolated from neck and head
                    keypoints_new[0] = (4 * keypoints_new[16] -
                                        keypoints_new[1]) / 3
                    # don't extrapolate the head top confidence score
                    keypoints_new[0, 2] = keypoints_new[16, 2]
                # arms and legs
                keypoints_new[2:14] = keypoints[[
                    6, 8, 10, 5, 7, 9, 12, 14, 16, 11, 13, 15
                ]]
            elif pose_det_dataset in ['TopDownAicDataset']:
                # head top is head top
                keypoints_new[0] = keypoints[12]
                # neck (bottom end of neck) is neck
                keypoints_new[1] = keypoints[13]
                # pelvis (root) is in the middle of l_hip and r_hip
                keypoints_new[14] = (keypoints[9] + keypoints[6]) / 2
                # spine (centre of torso) is in the middle of neck and root
                keypoints_new[15] = (keypoints_new[1] + keypoints_new[14]) / 2
                # head (spherical centre of head) is 7/12 the way from
                # neck (bottom end of neck) to head top
                keypoints_new[16] = (5 * keypoints[13] + 7 * keypoints[12]) / 12
                # arms and legs
                keypoints_new[2:14] = keypoints[0:12]
            elif pose_det_dataset in ['TopDownCrowdPoseDataset']:
                # head top is top_head
                keypoints_new[0] = keypoints[12]
                # neck (bottom end of neck) is in the middle of
                # l_shoulder and r_shoulder
                keypoints_new[1] = (keypoints[0] + keypoints[1]) / 2
                # pelvis (root) is in the middle of l_hip and r_hip
                keypoints_new[14] = (keypoints[7] + keypoints[6]) / 2
                # spine (centre of torso) is in the middle of neck and root
                keypoints_new[15] = (keypoints_new[1] + keypoints_new[14]) / 2
                # head (spherical centre of head) is 7/12 the way from
                # neck (bottom end of neck) to head top
                keypoints_new[16] = (5 * keypoints[13] + 7 * keypoints[12]) / 12
                # arms and legs
                keypoints_new[2:14] = keypoints[[
                    1, 3, 5, 0, 2, 4, 7, 9, 11, 6, 8, 10
                ]]

            else:
                raise NotImplementedError(
                    f'unsupported conversion between {pose_lift_dataset} and '
                    f'{pose_det_dataset}')

        return keypoints_new