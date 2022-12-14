#!/usr/bin/env python3
# VITA, EPFL
##################
#IMPORTS
##################
import rospy
import matplotlib.pyplot as plt
import time
import glob
from datetime import datetime
import os
import sys
import rospkg
rospack=rospkg.RosPack()
abs_path_to_loomo=rospack.get_path('loomo')
abs_path_to_tools=abs_path_to_loomo+"/scripts/tools"
sys.path.append(os.path.dirname(os.path.abspath(abs_path_to_tools)))
from tools import classes, classconverter
import csv
import cv2
import numpy as np
import PIL
from PIL import Image

#import custom perception packages
from perceptors import sot_perceptor, mot_perceptor
from detectors import yolov5_detector, pifpaf_detector
from trackers import mmtracking_sot
from tools.utils import Utils, Plotting, Transmission

#import msg type for the communication with pose_estimation node
from msg_types.msg import Bbox
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#Temporary to avoid warning due toe derecated command in stark tracker
import warnings
warnings.filterwarnings("ignore")

class Sender(object):
    def __init__(self):
        self.pub_bbox = rospy.Publisher('/Perception/bbox', Bbox, queue_size=1)
        self.pub_img = rospy.Publisher('/Perception/img', Image, queue_size=1)

        self.bridge=CvBridge()

    def send(self, bbox, opencv_img):
        #print(f"map global {map_global}")
        if not bbox:
            bbox=[0, 0, 0, 0]

        bbox_msg = classconverter.list2Bbox(bbox)
        self.pub_bbox.publish(bbox_msg)

        #try:
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(opencv_img, "bgr8"))
        #except CvBridgeError as e:
            #print(e)



def main():
    ###################################
    # Config from Launch File Arguments
    ####################################
    # Initialize ROS perception node
    rospy.init_node("perception")
    dt_perception = rospy.get_param("/dt_perception")
    mot_activated = rospy.get_param("mot_activated")
    rate = rospy.Rate(int(1/dt_perception))    
    PERCEPTION_FUNCTION = rospy.get_param("/PERCEPTION_FUNCTION")
    downscale = rospy.get_param("/downscale")
    detector_size=rospy.get_param("/detector_size")
    tracking_conf=rospy.get_param("/tracking_confidence")
    perception_vis=rospy.get_param("/visualization_percep")
    verbose_level=rospy.get_param("/verbose_percep")
    print(f"Verbose level is {verbose_level}")

    ###################################
    # Initialize Full detector
    ###################################
    # Initialize Detector Configuration
    # Set width, height and channel values for the received image --> Loomo image dimensions without downscale: 640x480x3
    if PERCEPTION_FUNCTION =="Default" or PERCEPTION_FUNCTION =="Stark":
        perceptor = sot_perceptor.SotPerceptor(width = 640, height = 480, downscale = downscale,
                                                detector = yolov5_detector.Yolov5Detector, detector_size="default", 
                                                tracker=mmtracking_sot.SotaTracker, tracker_model="Stark", tracking_conf=tracking_conf,
                                                mot_activated=mot_activated, type_input = "opencv", show=perception_vis, verbose=verbose_level)

    
    #################################
    # Initialize socket connections
    #################################
    #socket connection with Loomo => socket1: Receiver = Loomo's Camera; socket5: Sender = Detection Bbox 
    ip_address = rospy.get_param("/ip_address")
    socket1 = classes.SocketLoomo(8081, dt_perception, ip_address, perceptor.data_size)
    socket5 = classes.SocketLoomo(8085, dt_perception, ip_address, packer=25*'f ')

    #socket connection NeuroRestore
    #ip_address_neuro = rospy.get_param("/ip_address_neuro")
    #socket6 = classes.SocketLoomo(8086, dt_perception, ip_address_neuro, packer=13*'f ', sockettype="datagram")

    init = time.time()
    ##############################
    # Main Loop
    ###############################
    # Initialize perception transmission variables
    received_image = b''
    data_rcvd=False
    timer_started=False
    next_img=[]
    rospy.loginfo("Perception Node Ready")
    runtime_list=[]
    img_transmission_list=[]

    #initialize msg sender for pose estimation
    sender=Sender()

    while not rospy.is_shutdown():
        # start of main perception loop
        if not data_rcvd and not timer_started:
            # start timer for img transmission
            start_transmission = time.perf_counter()
            timer_started=True
        ################################
        # Receive Image from the Loomo
        ################################
        socket1.receiver(True)
        # safety in case img transmission is not synchronized
        received_image=Transmission.img_sync(next_img, received_image, socket1)
        next_img=[]

        # Image Processing only if we received the full package
        if len(received_image)==socket1.data_size:
            data_rcvd=True
            timer_started=False
            end_transmission=time.perf_counter()
            img_transmission_list.append(end_transmission-start_transmission)
            if verbose_level >=2:
                print(f"Elapsed time for Image Transmission: {(end_transmission-start_transmission)* 1e3:.1f}ms")
                
            # Reset perception variables
            input_img=received_image
            received_image = b''
            #############################
            # Inference on Received Image
            #############################
            bbox_list, image = perceptor.forward(input_img)
            if verbose_level >=1 :
                print("Perception: full bbox list:", bbox_list)

            ############################
            # Bounding Boxes Processing and Formatting
            #############################
            bbox = tuple()
            bbox_sot=None

            #SOT configurationwe have only 1 bbox at the end
            # Send bbox positions via socket to represent them in the Loomo
            if bbox_list:
                for i, bbox_indiv in enumerate(bbox_list):
                    if i==0 and not all(v == 0 for v in bbox_indiv):
                        #Target BBox send to pose estimation
                        bbox_sot = bbox + (bbox_indiv[0], bbox_indiv[1], bbox_indiv[2], bbox_indiv[3], float(True))#float(label_list[i][0]))
                    #Optional parameter: Scaling down bbox so that depth estimation is more precise inside small region
                    scale=1
                    bbox_indiv=Utils.bbox_scaling(bbox_indiv, scale)

                    # if i==0 and not all(v == 0 for v in bbox_indiv):
                    #Loomo with ADP App want the bbox in the following format: x_tl, y_tl, w, h
                    new_bbox=Utils.bbox_xcentycentwh_to_xtlytlwh(bbox_indiv)                    # else:

                    if all(v == 0 for v in bbox_indiv):
                        #handle case of lost target
                        bbox=bbox+(new_bbox[0], new_bbox[1], new_bbox[2], new_bbox[3], float(False))
                    else:
                        bbox=bbox+(new_bbox[0], new_bbox[1], new_bbox[2], new_bbox[3], float(True))

            else:
                bbox=bbox+(0.0, 0.0, 0.0, 0.0, float(False))            

            ###################################
            # Transmission to Loomo
            ###################################
            # Send bbox to the robot -> Camera tracking and motion controller algorithm
            bbox = bbox + (0.0,)*(25-len(bbox)) #this line just adding 20 times 0.0 after bbox which is 4+ 1 (label)
            if verbose_level>=3:
                print(f"debug bbox just before sending {bbox}")
            socket5.sender(bbox)

            # Send pose_estimation topic via ROS
            if bbox_sot and verbose_level>=1:
                print(f"bbox sent to pose est{bbox_sot}")
            sender.send(bbox_sot, image)

        #syncing Img transmission
        elif len(received_image) > socket1.data_size:
            next_img=Transmission.check_img_sync(received_image, socket, verbose_level)
            received_image = b''

        if data_rcvd:
            data_rcvd=False
            # Calculate node computation time
            end_perception=time.perf_counter()
            computation_time = end_perception - start_transmission
            runtime_list.append(computation_time)
            if verbose_level>=2:
                print(f"Elapsed time for full perception is: {computation_time* 1e3:.1f}ms")
            if computation_time > dt_perception:
                rospy.logwarn("Perception computation time higher than node period by " + str((computation_time-dt_perception)*1e3) + " ms")
            computation_time=0
            rate.sleep()
            
    print(f"Average runtime: {np.average(runtime_list)*1e3}ms, Image Transmission: {np.average(img_transmission_list)*1e3}ms")

if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass





            

            


