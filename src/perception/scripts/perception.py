#!/usr/bin/env python3
# VITA, EPFL


##################
#IMPORTS
##################
import rospy
import matplotlib.pyplot as plt
from Perception_Functions import detector, pifpaf_detector
import time
import glob
from datetime import datetime

import os
import sys
abs_path_to_tools = "/home/vita-w11/Autonomous_driving_pipeline/src/loomo/scripts/tools"
sys.path.append(os.path.dirname(os.path.abspath(abs_path_to_tools)))

abs_path_to_ADP="/home/theo/Autonomous_driving_pipeline/src/trackers"
sys.path.append(abs_path_to_ADP)
print(sys.path)
from tools import classes
import csv
import cv2
import numpy as np

now = datetime.now().strftime("%Y%m%d%H%M%S")

filename_data = "Stream_MR_" + str(now) + ".csv"
filename_video = "Stream_MR_" + str(now) + ".avi"
save_results = False

from perceptors import sot_perceptor
#from perceptors.sot_perceptor import SotPerceptor
from detectors import yolov5_detector
from trackers import mmtracking_sot





def main():

    ###################################
    # Config from Launch File Arguments
    ####################################
    # Initialize ROS perception node
    rospy.init_node("perception")
    dt_perception = rospy.get_param("/dt_perception")
    rate = rospy.Rate(int(1/dt_perception))    
    PERCEPTION_FUNCTION = rospy.get_param("/PERCEPTION_FUNCTION")
    downscale = rospy.get_param("/downscale")
    detector_size=rospy.get_param("/detector_size")
    tracking_conf=rospy.get_param("/tracking_confidence")
    perception_vis=rospy.get_param("/visualization_activated")

    ###################################
    # Initialize Full detector
    ###################################

    # Initialize Detector Configuration
    # Set width, height and channel values for the received image --> Loomo image dimensions: 80x60x3
    # Set downscale as the relation between Loomo image/detector expected image

    if PERCEPTION_FUNCTION == "Default":
        detection_image = classes.DetectorConfig(width = 80, height = 60, channels = 3, downscale = downscale,
                                                global_path = path_model,
                                                detector = detector.Detector(), load = True, type_input = "opencv")
    
    elif PERCEPTION_FUNCTION =="Openpifpaf":
        detection_image = classes.NewDetectorConfig(width = 640/downscale, height = 480/downscale, channels = 3, downscale = downscale,
                                                global_path = '',
                                                detector = pifpaf_detector.Detector_pifpaf(), load = False, type_input = "pil",
                                                save_video=save_results, filename_video=filename_video)

    elif PERCEPTION_FUNCTION =="Yolo":
        detection_image = classes.NewDetectorConfig(width = 640, height = 480, channels = 3, downscale = downscale,
                                                global_path = '',
                                                detector = yolo_detector.YoloDetector(), load = False, type_input = "opencv",
                                                save_video=save_results, filename_video=filename_video)
    
    elif PERCEPTION_FUNCTION =="Stark":
        perceptor = sot_perceptor.SotPerceptor(width = 640, height = 480, channels = 3, downscale = downscale,
                                                detector = yolov5_detector.Yolov5Detector(), detector_size="default", 
                                                tracker=mmtracking_sot.SotaTracker(), tracker_model="Stark", tracking_conf=tracking_conf,
                                                type_input = "opencv")


    #################################
    # Initialize socket connections
    #################################

    ip_address = rospy.get_param("/ip_address")
    ip_address_nicolo = rospy.get_param("/ip_address_nicolo")
    #try:
    socket1 = classes.SocketLoomo(8081, dt_perception, ip_address, detection_image.data_size)
    #except NameError:
        #detection_image=None
    socket5 = classes.SocketLoomo(8085, dt_perception, ip_address, packer=25*'f ')
    #socket6 = classes.SocketLoomo(8086, dt_perception, ip_address_nicolo, packer=13*'f ', sockettype="datagram")

    # Perception visualization tools activated?
    visualization = False
    init = time.time()


    ##############################
    # Main Loop
    ###############################

    # Initialize perception transmission variables
    net_received_length = 0
    received_image = b''

    rospy.loginfo("Perception Node Ready")

    with open(filename_data, 'w') as f:
        writer = csv.DictWriter(f, dialect='excel', fieldnames=['time', 'lhx', 'lhy', 'rhx', 'rhy', 'lkx', 'lky', 'rkx', 'rky', 'lax', 'lay', 'rax', 'ray'])
        writer.writeheader()

        while not rospy.is_shutdown():
            #rospy.loginfo("Perception loop")
            start = time.time()

            ################################
            # Receive Image from the Loomo
            ################################

            socket1.receiver(True)
            received_image += socket1.received_data
            net_received_length += len(socket1.received_data)
            #print(f"perceptions size 1: {net_received_length}, size 2: {socket1.data_size}")
            # If detector and received image size are the same
            if net_received_length == socket1.data_size:
                # Detect object/human
                #  inside the image

                #############################
                # Inference on Received Image
                #############################

                #when using Detector Config
                #bbox_list, label_list, bboxes_legs, image = detection_image.detect(received_image)
                bbox_list=SotPerceptor(received_image)
                print("in perception.py: bbox list:", bbox_list)

                

                if visualization:
                    plt.clf()
                    plt.plot(0.0, 0.0, ">k")

                ############################
                # Bounding Boxes Processing and Formatting
                #############################

                bbox = tuple()
                #print(bbox_list)
                for i in range(len(bbox_list)):
                    # Send bbox positions via socket to represent them in the Loomo
                    bbox_visu = bbox + (bbox_list[i][0], bbox_list[i][1], bbox_list[i][2], bbox_list[i][3], float(label_list[i][0]))
                    
                    #Loomo with Carlos App want the bbox in the following format: x_tl, y_tl, w, h
                    x_tl= bbox_list[i][0] - bbox_list[i][2]/2
                    y_tl= bbox_list[i][1] - bbox_list[i][3]/2
                    w= bbox_list[i][2]
                    h= bbox_list[i][3]
                    bbox= bbox+ (x_tl, y_tl, w, h, float(label_list[i][0]))

                ####################################
                #  BBox Visualization
                ####################################

                if bbox_visu is not None:
                    tl=(int(bbox_visu[0]-bbox_visu[2]/2), int(bbox_visu[1]+bbox_visu[3]/2)) #top-left corner
                    br= (int(bbox_visu[0]+bbox_visu[2]/2), int(bbox_visu[1]-bbox_visu[3]/2)) #bottom right corner
                    cv2.rectangle(image, tl, br, (255,0,0), 2)
                cv2.imshow('Camera Loomo',image)
                cv2.waitKey(1)
                

                ###################################
                # Transmission to Loomo
                ###################################

                # Send bbox to the robot -> Camera tracking and motion controller algorithm
                #bbox= (0.0, 0.0, 0.0, 0.0, 1.0) #trying to hand code 0 bbox to see if it's transmitted
                #bbox=(tl[0], tl[1], br[0], br[1])

                bbox = bbox + (0.0,)*(25-len(bbox)) #this line just adding 20 times 0.0 after bbox which is 4+ 1 (label)
                print(f"in perception bbox: {bbox}")
                socket5.sender(bbox)

                # pixel_legs = tuple()

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

                # Reset perception variables
                net_received_length = 0
                received_image = b''

            elif net_received_length > socket1.data_size:
                net_received_length = 0
                received_image = b''

            # Calculate node computation time
            computation_time = time.time() - start
            #print(f"perception computation time {computation_time}")

            
            # if computation_time > dt_perception:
            #     rospy.logwarn("Perception computation time higher than node period by " + str(computation_time-dt_perception) + " seconds")

            rate.sleep()

    f.close()

if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass





            

            


