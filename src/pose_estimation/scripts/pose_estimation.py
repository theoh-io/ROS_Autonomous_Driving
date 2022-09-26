#!/usr/bin/env python3
# VITA, EPFL

import rospy
import torch
import numpy as np
import time
from datetime import datetime
import os
import sys
import rospkg
rospack=rospkg.RosPack()
abs_path_to_loomo=rospack.get_path('loomo')
abs_path_to_tools=abs_path_to_loomo+"/scripts/tools"
sys.path.append(os.path.dirname(os.path.abspath(abs_path_to_tools)))
from tools import classconverter
from tools.utils import Utils
import cv2
import csv
from msg_types.msg import Bbox
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from keypoints3d import Keypoints3D

#Format of the Logging File
now = datetime.now().strftime("%Y%m%d%H%M%S")
filename_data = "Stream_MR_" + str(now) + ".csv"
filename_video = "Stream_MR_" + str(now) + ".avi"
path_output=os.path.abspath(abs_path_to_loomo+"/..")

def callback_bbox(data):

    global bbox

    bbox = classconverter.Bbox2list(data)
    
def callback_img(data):

    global cv_image
    bridge=CvBridge()

    try:
        cv_image=bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

#Class for file writer object supporting "with"
class KeypointLogging():
    def __init__(self, file_name, type, thresh, init_time):
        self.file_name=file_name
        #2D/3D keypoints logging
        self.type=type
        #confidence score threshold to suppress keypoint value
        self.thresh=thresh
        self.init_time=init_time
        self.file = open(self.file_name, 'w+')
        if self.type=="2D":
            self.writer=csv.DictWriter(self.file, dialect='excel', fieldnames=['time', 'lhx', 'lhy', 'rhx', 'rhy', 'lkx', 'lky', 'rkx', 'rky', 'lax', 'lay', 'rax', 'ray'])
            self.writer.writeheader()
        else:
            print("type of keypoint logger not implemented !!")
        
    def save(self, keypoints):
        if self.type=="2D":
            Utils.save_2Dkeypoints(keypoints, self.thresh, self.writer, self.init_time)
        else:
            print("type of keypoint logger not implemented !!")

    def close(self):
        self.file.close()
    
def main():
    # Initialize ROS prediction node
    rospy.init_node("pose_estimation")
    dt_pose_estimation = rospy.get_param("/dt_pose_estimation")
    downscale = rospy.get_param("/downscale")
    keypoints_activ= rospy.get_param("/keypoints_activated")
    show3D= rospy.get_param("/visualization_3D_activated")
    save_video_keypoints=rospy.get_param("/save_keypoints_vid")
    keypoints_logging=rospy.get_param("/keypoints_logging")
    verbose= rospy.get_param("/verbose_keypoints")
    resolution=(int(640/downscale), int(480/downscale))
    device=torch.device('cuda:0')
    rate = rospy.Rate(int(1/dt_pose_estimation))
    
    #subscriber perception topic
    global bbox
    global cv_image
    cv_image=None
    bbox=[0, 0, 0, 0]
    sub_bbox = rospy.Subscriber('/Perception/bbox', Bbox, callback_bbox, queue_size = 1)
    sub_img= rospy.Subscriber('/Perception/img', Image, callback_img, queue_size=1)
    
    if keypoints_activ:
        keypoints3D=Keypoints3D(device, resolution, show3D, save_video_keypoints)

    #logging
    if keypoints_logging:
        path_data=path_output+"/"+filename_data
    else:
        #save the csv in .ros folder so that it doesn't annoy 
        path_data=os.getcwd()+"/log.csv"
    init=time.time()
    logger=KeypointLogging(path_data, "2D", 0.25, init)


    runtime_list=[]
    computation_time=0
    while not rospy.is_shutdown():
        #print(bbox)
        if cv_image is not None and keypoints_activ:
            if bbox:
                tic1 = time.perf_counter()
                results_keypoints=keypoints3D.inference_3Dkeypoints(cv_image, bbox)
                toc1 = time.perf_counter()
                # cv2.imshow("pose est", cv_image)
                # cv2.waitKey(1)
                if keypoints_logging and results_keypoints:
                    logger.save(results_keypoints)
                if verbose:
                    print(f"Elapsed time for 3D keypoints forward pass: {(toc1 - tic1) * 1e3:.1f}ms")

                toc2=time.perf_counter()
                computation_time=toc2-tic1
                if verbose:
                    print(f"Elapsed time for full 3D pose estimation: {computation_time * 1e3:.1f}ms")
                    runtime_list.append(computation_time)
            if computation_time > dt_pose_estimation:
                rospy.logwarn("Perception computation time higher than node period by " + str((computation_time-dt_pose_estimation)*1e3) + " ms")
        rate.sleep()

    print(f"Average 3D pose runtime: {np.average(runtime_list)*1e3}ms")
    logger.close()

if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass



#####################################################
# Previous code for communication with Neuro Device #
#####################################################

#     #socket connection NeuroRestore
#     #ip_address_neuro = rospy.get_param("/ip_address_neuro")
#     #socket6 = classes.SocketLoomo(8086, dt_perception, ip_address_neuro, packer=13*'f ', sockettype="datagram")


# for i in range(len(bboxes_legs)):
#     # Send bbox positions via socket to represent them in the Loomo
#     pixel_legs = pixel_legs + (bboxes_legs[i][0], bboxes_legs[i][1])

# pixel_legs = tuple([time.time()-init]) + pixel_legs
# pl = pixel_legs[:13]

# if pl[1]!=0.0:
#     # Send information to NeuroRestore Stimulation algorithm
#     socket6.sender(pl)

# if save_results:
#     dict_legs = {'time': pl[0], 'lhx': pl[1], 'lhy': pl[2], 'rhx': pl[3], 'rhy': pl[4], 'lkx': pl[5], 'lky': pl[6], 'rkx': pl[7], 'rky': pl[8], 'lax': pl[9], 'lay': pl[10], 'rax': pl[11], 'ray': pl[12]}
#     writer.writerow(dict_legs)




            

            


