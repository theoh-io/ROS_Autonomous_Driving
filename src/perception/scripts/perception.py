#!/usr/bin/env python3
# VITA, EPFL
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
from tools import classes
import csv
import cv2
import numpy as np

now = datetime.now().strftime("%Y%m%d%H%M%S")

filename_data = "Stream_MR_" + str(now) + ".csv"
filename_video = "Stream_MR_" + str(now) + ".avi"
save_results = True

def main():
    # Initialize ROS perception node
    rospy.init_node("perception")
    dt_perception = rospy.get_param("/dt_perception")
    rate = rospy.Rate(int(1/dt_perception))
    PERCEPTION_FUNCTION = rospy.get_param("/PERCEPTION_FUNCTION")
    path_model = rospy.get_param("/model_perception_path")
    downscale = rospy.get_param("/downscale")
    # Initialize Detector Configuration
    # Set width, height and channel values for the received image --> Loomo image dimensions: 80x60x3
    # Set downscale as the relation between Loomo image/detector expected image

    if PERCEPTION_FUNCTION == "Default":
        detection_image = classes.DetectorConfig(width = 80, height = 60, channels = 3, downscale = 1,
                                                global_path = path_model,
                                                detector = detector.Detector(), load = True, type_input = "opencv")
    
    elif PERCEPTION_FUNCTION =="Openpifpaf":
        detection_image = classes.DetectorConfig(width = 640/downscale, height = 480/downscale, channels = 3, downscale = 1,
                                                global_path = '',
                                                detector = pifpaf_detector.Detector_pifpaf(), load = False, type_input = "pil",
                                                save_video=save_results, filename_video=filename_video)


    # Initialize socket connections
    ip_address = rospy.get_param("/ip_address")
    ip_address_nicolo = rospy.get_param("/ip_address_nicolo")
    socket1 = classes.SocketLoomo(8081, dt_perception, ip_address, detection_image.data_size)
    socket5 = classes.SocketLoomo(8085, dt_perception, ip_address, packer=25*'f ')
    socket6 = classes.SocketLoomo(8086, dt_perception, ip_address_nicolo, packer=13*'f ', sockettype="datagram")

    # Perception visualization tools activated?
    visualization = False
    init = time.time()

    # Initialize perception variables
    net_received_length = 0
    received_image = b''

    rospy.loginfo("Perception Node Ready")

    with open(filename_data, 'w') as f:
        writer = csv.DictWriter(f, dialect='excel', fieldnames=['time', 'lhx', 'lhy', 'rhx', 'rhy', 'lkx', 'lky', 'rkx', 'rky', 'lax', 'lay', 'rax', 'ray'])
        writer.writeheader()

        while not rospy.is_shutdown():
            start = time.time()

            # Receive Image from the Loomo
            socket1.receiver(True)
            received_image += socket1.received_data
            net_received_length += len(socket1.received_data)

            # If detector and received image size are the same
            if net_received_length == socket1.data_size:
                # Detect object/human
                #  inside the image
                bbox_list, label_list, bboxes_legs = detection_image.detect(received_image)

                if visualization:
                    plt.clf()
                    plt.plot(0.0, 0.0, ">k")

                bbox = tuple()
                
                for i in range(len(bbox_list)):
                    # Send bbox positions via socket to represent them in the Loomo
                    bbox = bbox + (bbox_list[i][0], bbox_list[i][1], bbox_list[i][2], bbox_list[i][3], float(label_list[i][0]))
                    
                # Send bbox to the robot -> Camera tracking and motion controller algorithm
                bbox = bbox + (0.0,)*(25-len(bbox))
                socket5.sender(bbox)

                pixel_legs = tuple()

                for i in range(len(bboxes_legs)):
                    # Send bbox positions via socket to represent them in the Loomo
                    pixel_legs = pixel_legs + (bboxes_legs[i][0], bboxes_legs[i][1])

                pixel_legs = tuple([time.time()-init]) + pixel_legs
                pl = pixel_legs[:13]
                
                if pl[1]!=0.0:
                    # Send information to NeuroRestore Stimulation algorithm
                    socket6.sender(pl)

                if save_results:
                    dict_legs = {'time': pl[0], 'lhx': pl[1], 'lhy': pl[2], 'rhx': pl[3], 'rhy': pl[4], 'lkx': pl[5], 'lky': pl[6], 'rkx': pl[7], 'rky': pl[8], 'lax': pl[9], 'lay': pl[10], 'rax': pl[11], 'ray': pl[12]}
                    writer.writerow(dict_legs)

                # Reset perception variables
                net_received_length = 0
                received_image = b''

            elif net_received_length > socket1.data_size:
                net_received_length = 0
                received_image = b''

            # Calculate node computation time
            computation_time = time.time() - start

            #if computation_time > dt_perception:
                #rospy.logwarn("Perception computation time higher than node period by " + str(computation_time-dt_perception) + " seconds")

            rate.sleep()

    f.close()

if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass





            

            


