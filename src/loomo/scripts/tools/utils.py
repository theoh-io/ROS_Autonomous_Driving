import numpy as np
import math
import torch
import matplotlib.pyplot as plt
import collections
import cv2
from PIL import Image
import time


class Utils():
    # @staticmethod
    # #FIX: use BBOX format to crop, output format tensor image ?
    # def crop_img_bbox(bbox_list: list, img: np.ndarray):
    #     img_list=[]
    #     print(f"bbox_list: {bbox_list}")
    #     print(type(img))
    #     if bbox_list is not None:
    #         for i in range(bbox_list.shape[0]):
    #             bbox_indiv=bbox_list[i]
    #             print(f"bbox_indiv{bbox_indiv}")
    #             crop_img=img[int((bbox_indiv[1]-bbox_indiv[3]/2)):int((bbox_indiv[1]+bbox_indiv[3]/2)), int((bbox_indiv[0]-bbox_indiv[2]/2)):int((bbox_indiv[0]+bbox_indiv[2]/2))]
    #             #to apply the normalization need a PIL image
    #             # PIL RGB while CV is BGR.
    #             #crop_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB)
    #             #crop_img = Image.fromarray(crop_img)
    #             #tensor_img_indiv=image_processing(crop_img)

    #             tensor_img_indiv=torch.unsqueeze(torch.from_numpy(crop_img), 0)
    #             img_list.append(tensor_img_indiv)
    #         #print()
    #         np_img_list=np.asarray(img_list)
    #         print(img_list.shape)
    #         tensor_img=torch.from_numpy(np_img_list)
    #         #tensor_img=torch.cat(img_list)
    #         return tensor_img
    #     else:
    #         return None

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
    def adapt_bbox(bbox):
        #adapt in case corner outside of screen
        x_tl= bbox[0] - bbox[2]/2
        y_tl= bbox[1] - bbox[3]/2
        w= bbox[2]
        h= bbox[3]
        if x_tl<0:
            w+=x_tl
            x_tl=0
        if y_tl<0:
            h+=y_tl
            y_tl=0
        return [x_tl, y_tl, w, h]

        

    @staticmethod
    def bbox_xcentycentwh_to_xtlytlwh(bbox):
        #convert from (xcenter,ycenter,width,height) to (x_topleft,y_topleft, width, height)
        #Fixed 0 0 if outside the cam
        x_tl= bbox[0] - bbox[2]/2
        y_tl= bbox[1] - bbox[3]/2
        w= bbox[2]
        h= bbox[3]
        new_bbox=[x_tl, y_tl, w, h]
        
        if x_tl<0 or y_tl <0:
            new_bbox=Utils.adapt_bbox(bbox)
        
        
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
    def mult_bbox_vis(bbox_list, image):
        #input format (xcenter,ycenter,width,height) => cv2 (x_tl,y_tl,x_br,y_br)
        if bbox_list:
            for bbox in bbox_list:
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

    @staticmethod
    def save_2Dkeypoints(keypoints_result, threshold, writer, init_time):
        #format of keypoints depends on datasets used check convert_keypoints function
        #pose_lift_dataset= Body3DH36MDataset
        #pose_det_dataset= TopDownCocoDataset
        #0:pelvis, 1: right hip, 2: rknee, 3:rankle, 4:left hip, 5:lknee, 6:la, 7:spine
        #8:thorax, 9:nose, 10:head, 11:left shoulder,12:lelbow, 13:lw, 14: rs, 15:re, 16:rw 
        k2D=keypoints_result["keypoints"]
        #if score is too low replace values by None
        for pt in k2D:
            if pt[2]<threshold:
                pt[0]=None
                pt[1]=None
        curr_time=time.time()-init_time
        dict_legs={'time': curr_time, 'lhx': k2D[4][0], 'lhy': k2D[4][1], 'rhx': k2D[1][0], 'rhy': k2D[4][1], 'lkx': k2D[5][0], 'lky': k2D[5][1], 'rkx': k2D[2][0], 'rky': k2D[2][1], 'lax': k2D[6][0], 'lay': k2D[6][1], 'rax': k2D[3][0], 'ray': k2D[3][1]}
        writer.writerow(dict_legs)

    @staticmethod
    def save_3Dkeypoints(keypoints_result, writer):
        keypoints3D=keypoints_results["keypoints"]
    
    @staticmethod
    def calculate_distance(object1, object2=[0.0, 0.0]):
        '''
        Calculate distance between 2 objects
        '''
        return math.sqrt((object1[0]-object2[0])**2 + (object1[1]-object2[1])**2)


    @staticmethod
    def minimum_distance(state, array_path):
        '''
        Calculate the index of the minim distance between array_path and state
        '''
        distances = []

        for e in array_path:
            dist_x = (e[0] - state[0])**2
            dist_y = (e[1] - state[1])**2
            dist_heading = (e[2] - state[2])**2
            dist_total = dist_x + dist_y + dist_heading
            distances.append(dist_total)

        val, idx = min((val, idx) for (idx, val) in enumerate(distances))

        return idx

    # Apply Kinematic Restrictions to the planner, and adapt it for MPC.
    @staticmethod
    def MPC_Planner_restrictions(mobile_robot, points, v, t):
        e = v*t
        plan = [[0.0, 0.0, 0.0, 0.0]]
        dist = 0.0
        it = 2
        x_ant = 0.0
        y_ant = 0.0
        heading_ant = 0.0

        for i in range(1,80):
            
            while dist < e and it < len(points):
                distAB = Utils.calculate_distance(points[it-2], points[it-1])
                dist = dist + distAB
                it = it + 1

            if dist >= e:
                it = it - 1
                d = dist - e
                prop = d/distAB
                x = points[it-1][0] - prop*(points[it-1][0] - points[it-2][0])
                y = points[it-1][1] - prop*(points[it-1][1] - points[it-2][1])
                heading = math.atan2((y - y_ant),(x - x_ant))

                if abs(heading-heading_ant) > mobile_robot.w_max * t:
                    heading = heading_ant + np.sign(heading-heading_ant) * mobile_robot.w_max * t
                    x = x_ant + e * np.cos(heading)
                    y = y_ant + e * np.sin(heading)

                plan.append([x, y, heading, v])
                dist = d
                x_ant = x
                y_ant = y
                heading_ant = heading
                it = it + 1

        return plan

    # Apply Kinematic Restrictions to the planner, and adapt it for MPC.
    @staticmethod
    def MPC_Planner_restrictions_CHUV_straight(mobile_robot, points, v, t, N, x0=[0.0, 0.0, 0.0, 0.0]):
        
        if len(points)>1:
            v = min(abs((points[-1][0]-x0[0])/(t*N)), v)
        e = v*t
        plan = [x0]
        dist = 0.0
        it = 2
        x_ant = 0.0
        y_ant = 0.0
        heading_ant = 0.0

        for i in range(1,80):
            #compute number of points in plans for 1 control step
            while dist < e and it < len(points):
                distAB = Utils.calculate_distance(points[it-2], points[it-1])
                dist = dist + distAB
                it = it + 1
            #supplement distance to reach discrete path planning objective in x, y, heading
            if dist >= e:
                it = it - 1
                d = dist - e
                prop = d/distAB
                x = points[it-1][0] - prop*(points[it-1][0] - points[it-2][0])
                y = points[it-1][1] - prop*(points[it-1][1] - points[it-2][1])
                heading = math.atan2((y - y_ant),(x - x_ant))
                v = abs(v)

                # condition if rate of orientation > rotation possible by loomo in one control step
                if abs(heading-heading_ant) > mobile_robot.w_max * t:
                    heading = 0.0
                    x = x_ant - e * np.cos(heading)
                    y = y_ant - e * np.sin(heading)
                    v = -v

                plan.append([x, y, heading, v])
                dist = d
                x_ant = x
                y_ant = y
                heading_ant = heading
                it = it + 1

        return plan

    @staticmethod
    def MPC_Planner_restrictions_CHUV_curvilinear(mobile_robot, points, speed, t, x0=[0.0, 0.0, 0.0, 0.0]):
        e = speed*t
        x0.append(0.0)
        plan = [x0]
        dist = 0.0
        it = 2
        x_ant = x0[0]
        y_ant = x0[1]
        heading_ant = x0[2]

        for i in range(1,80):
            
            while dist < e and it < len(points):
                distAB = Utils.calculate_distance(points[it-2], points[it-1])
                dist = dist + distAB
                it = it + 1

            if dist >= e:
                it = it - 1
                d = dist - e
                prop = d/distAB
                x = points[it-1][0] - prop*(points[it-1][0] - points[it-2][0])
                y = points[it-1][1] - prop*(points[it-1][1] - points[it-2][1])
                heading = math.atan2((y - y_ant),(x - x_ant))
                heading_act = heading
                v = speed
                
                #if abs(heading-x0[2]) <= math.pi/2:
                x = x_ant + e * np.cos(heading)
                y = y_ant + e * np.sin(heading)

                if abs(heading_act-heading_ant) > mobile_robot.w_max * t:
                    heading = heading_ant + np.sign(heading-heading_ant) * mobile_robot.w_max * t
                    v = 0.0
                    x = x_ant
                    y = y_ant

                #elif abs(heading-x0[2]) > math.pi/2:
                    #v = -v
                    #x = x_ant - e * np.cos(heading)
                    #y = y_ant - e * np.sin(heading)
                    #heading = heading - np.sign(heading) * math.pi

                    #if abs(heading_act-heading_ant) > mobile_robot.w_max * t:
                        #heading = heading_ant + np.sign(heading-heading_ant) * mobile_robot.w_max * t

                plan.append([x, y, heading, v])
                dist = d
                x_ant = x
                y_ant = y
                heading_ant = heading
                it = it + 1

        return plan

    @staticmethod
    def new_heading_required(past_person, person):
        return (Utils.calculate_distance(past_person, person)>0.3)

    @staticmethod
    def GetClockAngle(v1, v2):
        # Product of 2 vector modules
        TheNorm = np.linalg.norm(v1)*np.linalg.norm(v2)
        # Cross product
        rho =  np.arcsin(np.cross(v1, v2)/TheNorm)
        # Dot multiplication
        theta = np.arccos(np.dot(v1,v2)/TheNorm)
        
        if rho < 0:
            return - theta
        else:
            return theta
    
    @staticmethod
    def add_detections_to_past(pos_detections, past_pos_detections, past_number): # past_pos_detections = [deque([x1(t-past_n),y1(t-past_n)],...,[x1(t-1),y1(t-1)]),...,deque([xN(t-past_n),yN(t-past_n)],...,[xN(t-1),yN(t-1)])]
        '''
        Add new detections to the past detections buffer
        '''
        list_idx = []
        past_for_predictions = []
        objects_detected = False

        if len(pos_detections)>0:
            objects_detected = True

        if objects_detected:
            if len(pos_detections[0])>0:

                for detection in pos_detections:
                    list_idx.append(detection[2]-1)

                    if detection[2] > len(past_pos_detections):
                        d = collections.deque(maxlen=past_number)

                        for i in range(past_number):
                            d.append([detection[0],detection[1],detection[2]])

                        past_pos_detections.append(d)

                    else:
                        past_pos_detections[detection[2]-1].append([detection[0],detection[1], detection[2]])

        for idx in range(len(past_pos_detections)):

            if idx not in list_idx:
                past_pos_detections[idx].append([past_pos_detections[idx][past_number-1][0], past_pos_detections[idx][past_number-1][1], idx+1])
            
            else:
                past_for_predictions.append(list(past_pos_detections[idx]))

        return past_pos_detections, past_for_predictions






class Plotting():
    @staticmethod
    def plot_circle(x, y, size, color="b-"):
        '''
        Plot cicrles --> Object mapping, detection and prediction plot.
        '''
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

class Transmission():
    @staticmethod
    def img_sync(excess_img, current_img, socket):
    #to avoid an offset in the img transmission due to desynchronization of transmission
        if excess_img:
            current_img += excess_img
        current_img += socket.received_data
        while len(current_img) > socket.data_size:
            current_img=current_img[socket.data_size:]
        return current_img

    @staticmethod
    def check_img_sync(input_img, socket, verbose_level):
        if verbose_level >=4 :
            print("Image Transmission not synced !!")
        next_img=input_img[socket.data_size:]
        while len(next_img) > socket1.data_size:
            next_img=input_img[socket1.data_size:]
        if verbose_level>=4:
            print(f"surplus {len(next_img)}")
        return next_img

