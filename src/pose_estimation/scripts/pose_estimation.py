#!/usr/bin/env python3
# VITA, EPFL

# ##################
# #IMPORTS
# ##################
# import rospy
# import matplotlib.pyplot as plt
# import time
# import glob
# from datetime import datetime
# import os
# import sys
# import rospkg
# rospack=rospkg.RosPack()
# abs_path_to_loomo=rospack.get_path('loomo')
# abs_path_to_tools=abs_path_to_loomo+"/scripts/tools"
# sys.path.append(os.path.dirname(os.path.abspath(abs_path_to_tools)))
# from tools import classes
# import csv
# import cv2
# import numpy as np
# import PIL
# from PIL import Image

# #Format of the Logging File
# now = datetime.now().strftime("%Y%m%d%H%M%S")
# filename_data = "Stream_MR_" + str(now) + ".csv"
# filename_video = "Stream_MR_" + str(now) + ".avi"
# path_output=os.path.abspath(abs_path_to_loomo+"/..")

# #import custom perception packages
# from perceptors import sot_perceptor, mot_perceptor
# from detectors import yolov5_detector, pifpaf_detector
# from trackers import mmtracking_sot
# from tools.utils import Utils, Plotting, Transmission




# def main():
#     ###################################
#     # Config from Launch File Arguments
#     ####################################
#     # Initialize ROS perception node
#     rospy.init_node("perception")
#     dt_perception = rospy.get_param("/dt_perception")
#     rate = rospy.Rate(int(1/dt_perception))    
#     PERCEPTION_FUNCTION = rospy.get_param("/PERCEPTION_FUNCTION")
#     downscale = rospy.get_param("/downscale")
#     detector_size=rospy.get_param("/detector_size")
#     tracking_conf=rospy.get_param("/tracking_confidence")
#     keypoints_activated=rospy.get_param("/keypoints_activated")
#     save_keypoints=rospy.get_param("/save_keypoints_vid")
#     keypoints_logging=rospy.get_param("/keypoints_logging")
#     perception_vis=rospy.get_param("/visualization_percep")
#     keypoints_vis=rospy.get_param("/visualization_3D_activated")
#     verbose_level=rospy.get_param("/verbose_percep")
#     print(f"Verbose level is {verbose_level}")


#     if keypoints_logging:
#         path_data=path_output+"/"+filename_data
#     else:
#         #save the csv in .ros folder so that it doesn't annoy 
#         path_data=os.getcwd()+"/log.csv"
#     ###################################
#     # Initialize Full detector
#     ###################################
#     # Initialize Detector Configuration
#     # Set width, height and channel values for the received image --> Loomo image dimensions without downscale: 640x480x3
#     if PERCEPTION_FUNCTION =="Default" or PERCEPTION_FUNCTION =="Stark":
#         perceptor = sot_perceptor.SotPerceptor(width = 640, height = 480, channels = 3, downscale = downscale,
#                                                 detector = yolov5_detector.Yolov5Detector, detector_size="default", 
#                                                 tracker=mmtracking_sot.SotaTracker, tracker_model="Stark", tracking_conf=tracking_conf,
#                                                 type_input = "opencv", keypoints=keypoints_activated, save_video_keypoints=save_keypoints, 
#                                                 show=perception_vis, show3D=keypoints_vis, verbose=verbose_level)
#     #################################
#     # Initialize socket connections
#     #################################
#     #socket connection with Loomo => socket1: Receiver = Loomo's Camera; socket5: Sender = Detection Bbox 
#     ip_address = rospy.get_param("/ip_address")
#     socket1 = classes.SocketLoomo(8081, dt_perception, ip_address, perceptor.data_size)
#     socket5 = classes.SocketLoomo(8085, dt_perception, ip_address, packer=25*'f ')

#     #socket connection NeuroRestore
#     #ip_address_neuro = rospy.get_param("/ip_address_neuro")
#     #socket6 = classes.SocketLoomo(8086, dt_perception, ip_address_neuro, packer=13*'f ', sockettype="datagram")

#     init = time.time()
#     ##############################
#     # Main Loop
#     ###############################
#     # Initialize perception transmission variables
#     received_image = b''
#     data_rcvd=False
#     timer_started=False
#     next_img=[]
#     rospy.loginfo("Perception Node Ready")
#     runtime_list=[]
#     img_transmission_list=[]

#     #open csv file for keypoint logging
#     with open(path_data, 'w+') as f:
#         writer = csv.DictWriter(f, dialect='excel', fieldnames=['time', 'lhx', 'lhy', 'rhx', 'rhy', 'lkx', 'lky', 'rkx', 'rky', 'lax', 'lay', 'rax', 'ray'])
#         writer.writeheader()

#         while not rospy.is_shutdown():
#             # start of main perception loop
#             if not data_rcvd and not timer_started:
#                 # start timer for img transmission
#                 start_transmission = time.perf_counter()
#                 timer_started=True
#             ################################
#             # Receive Image from the Loomo
#             ################################
#             socket1.receiver(True)
#             # safety in case img transmission is not synchronized
#             received_image=Transmission.img_sync(next_img, received_image, socket1)
#             next_img=[]

#             # Image Processing only if we received the full package
#             if len(received_image)==socket1.data_size:
#                 data_rcvd=True
#                 timer_started=False
#                 end_transmission=time.perf_counter()
#                 img_transmission_list.append(end_transmission-start_transmission)
#                 if verbose_level >=2:
#                     print(f"Elapsed time for Image Transmission: {(end_transmission-start_transmission)* 1e3:.1f}ms")
                    
#                 # Reset perception variables
#                 input_img=received_image
#                 received_image = b''
#                 #############################
#                 # Inference on Received Image
#                 #############################
#                 bbox_list, label_list, results_keypoints, image = perceptor.forward(input_img)
#                 if verbose_level >=1 :
#                     print("Perception: bbox list:", bbox_list)

#                 ############################
#                 # Bounding Boxes Processing and Formatting
#                 #############################
#                 bbox = tuple()
#                 bbox_visu=None

#                 #SOT configurationwe have only 1 bbox at the end
#                 if bbox_list and np.asarray(bbox_list).ndim==1:
#                     # Send bbox positions via socket to represent them in the Loomo
#                     bbox_visu = bbox + (bbox_list[0], bbox_list[1], bbox_list[2], bbox_list[3], float(True))#float(label_list[i][0]))
#                     #Optional parameter: Scaling down bbox so that depth estimation is more precise inside small region
#                     scale=1
#                     bbox_list=Utils.bbox_scaling(bbox_list, scale)
#                     #Loomo with ADP App want the bbox in the following format: x_tl, y_tl, w, h
#                     new_bbox=Utils.bbox_xcentycentwh_to_xtlytlwh(bbox_list)
#                     #new_bbox=bbox_list
#                     bbox= bbox+(new_bbox[0], new_bbox[1], new_bbox[2], new_bbox[3], float(True))

#                 else:
#                     bbox=bbox+(0.0, 0.0, 0.0, 0.0, float(False))            

#                 ###################################
#                 # Transmission to Loomo
#                 ###################################
#                 # Send bbox to the robot -> Camera tracking and motion controller algorithm
#                 bbox = bbox + (0.0,)*(25-len(bbox)) #this line just adding 20 times 0.0 after bbox which is 4+ 1 (label)
#                 socket5.sender(bbox)

#                 ####################################
#                 # Keypoints Logging & Transmission #
#                 ####################################
#                 # pixel_legs = tuple()
#                 if keypoints_logging and results_keypoints:
#                     Utils.save_2Dkeypoints(results_keypoints, 0.25, writer, init)

#                 #####################################################
#                 # Previous code for communication with Neuro Device #
#                 #####################################################
                
#                 # for i in range(len(bboxes_legs)):
#                 #     # Send bbox positions via socket to represent them in the Loomo
#                 #     pixel_legs = pixel_legs + (bboxes_legs[i][0], bboxes_legs[i][1])

#                 # pixel_legs = tuple([time.time()-init]) + pixel_legs
#                 # pl = pixel_legs[:13]
                
#                 # if pl[1]!=0.0:
#                 #     # Send information to NeuroRestore Stimulation algorithm
#                 #     socket6.sender(pl)

#                 # if save_results:
#                 #     dict_legs = {'time': pl[0], 'lhx': pl[1], 'lhy': pl[2], 'rhx': pl[3], 'rhy': pl[4], 'lkx': pl[5], 'lky': pl[6], 'rkx': pl[7], 'rky': pl[8], 'lax': pl[9], 'lay': pl[10], 'rax': pl[11], 'ray': pl[12]}
#                 #     writer.writerow(dict_legs)

#             #syncing Img transmission
#             elif len(received_image) > socket1.data_size:
#                 next_img=Transmission.check_img_sync(received_image, socket, verbose_level)
#                 received_image = b''

#             if data_rcvd:
#                 data_rcvd=False
#                 # Calculate node computation time
#                 end_perception=time.perf_counter()
#                 computation_time = end_perception - start_transmission
#                 runtime_list.append(computation_time)
#                 if verbose_level>=2:
#                     print(f"Elapsed time for full perception is: {computation_time* 1e3:.1f}ms")
#                 if computation_time > dt_perception:
#                     rospy.logwarn("Perception computation time higher than node period by " + str((computation_time-dt_perception)*1e3) + " ms")
#                 computation_time=0
#                 rate.sleep()
                
#     print(f"Average runtime: {np.average(runtime_list)*1e3}ms, Image Transmission: {np.average(img_transmission_list)*1e3}ms")
#     f.close()
import rospy
import torch
import time
import os
import sys
import rospkg
rospack=rospkg.RosPack()
abs_path_to_loomo=rospack.get_path('loomo')
abs_path_to_tools=abs_path_to_loomo+"/scripts/tools"
sys.path.append(os.path.dirname(os.path.abspath(abs_path_to_tools)))
from tools import classconverter
import cv2
from msg_types.msg import Bbox
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from keypoints3d import Keypoints3D


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

    

def main():
    # Initialize ROS prediction node
    rospy.init_node("pose_estimation")
    dt_pose_estimation = rospy.get_param("/dt_pose_estimation")
    rate = rospy.Rate(int(1/dt_pose_estimation))
    #subscriber perception topic
    global bbox
    global cv_image
    cv_image=None
    bbox=[0, 0, 0, 0]
    sub_bbox = rospy.Subscriber('/Perception/bbox', Bbox, callback_bbox, queue_size = 1)
    sub_img= rospy.Subscriber('/Perception/img', Image, callback_img, queue_size=1)
    
    #3D Pose Estimation
    keypoints_activ=True
    show3D=True
    save_video_keypoints=None
    downscale = 2
    resolution=(int(640/downscale), int(480/downscale))
    device=torch.device('cuda:0')
    verbose=True


    if keypoints_activ:
        keypoints3D=Keypoints3D(device, resolution, show3D, save_video_keypoints)

    while not rospy.is_shutdown():
        print("hello World")
        print(bbox)
        if cv_image is not None:
            cv2.imshow("pose est", cv_image)
            cv2.waitKey(1)
            if keypoints_activ and bbox:
                tic = time.perf_counter()
                results_keypoints=keypoints3D.inference_3Dkeypoints(cv_image, bbox)
                toc = time.perf_counter()
                if verbose:
                    print(f"Elapsed time for 3D keypoints forward pass: {(toc - tic) * 1e3:.1f}ms")


        rate.sleep()


if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass





            

            


