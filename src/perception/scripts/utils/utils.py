import numpy as np

#FrameGraber
#Bbox Formatting

#bbox cropping

class Utils():
    @staticmethod:
    #FIX: use BBOX format to crop, output format tensor image ?
    def crop_img_bbox(bbox_list: list, img: np.ndarray):
        img_list=[]
        if bbox_list is not None:
            for i in range(bbox_list.shape[0]):
                bbox_indiv=bbox_list[i]
                crop_img=np.array(img[int((bbox_indiv[1]-bbox_indiv[3]/2)):int((bbox_indiv[1]+bbox_indiv[3]/2)), int((bbox_indiv[0]-bbox_indiv[2]/2)):int((bbox_indiv[0]+bbox_indiv[2]/2))])
                #to apply the normalization need a PIL image
                # PIL RGB while CV is BGR.
                crop_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB)
                crop_img = Image.fromarray(crop_img)
                #tensor_img_indiv=image_processing(crop_img)
                tensor_img_indiv=torch.unsqueeze(tensor_img_indiv, 0)
                img_list.append(tensor_img_indiv)
            tensor_img=torch.cat(img_list)
            return tensor_img
        else:
            return None