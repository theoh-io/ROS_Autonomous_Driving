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