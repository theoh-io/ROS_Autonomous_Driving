#!/usr/bin/env python3
# VITA, EPFL
import rospy
import matplotlib.pyplot as plt
from Perception_Functions import detector, pifpaf_detector
import time

import os
import sys
abs_path_to_tools = rospy.get_param("/abs_path_to_tools")
sys.path.append(os.path.dirname(os.path.abspath(abs_path_to_tools)))
from tools import classes


def main():
    # Initialize ROS perception node
    rospy.init_node("perception")
    dt_perception = rospy.get_param("/dt_perception")
    rate = rospy.Rate(int(1/dt_perception))
    PERCEPTION_FUNCTION = rospy.get_param("/PERCEPTION_FUNCTION")
    path_model = rospy.get_param("/model_perception_path")
    # Initialize Detector Configuration
    # Set width, height and channel values for the received image --> Loomo image dimensions: 80x60x3
    # Set downscale as the relation between Loomo image/detector expected image

    if PERCEPTION_FUNCTION == "Default":
        detection_image = classes.DetectorConfig(width = 80, height = 60, channels = 3, downscale = 1,
                                                global_path = path_model,
                                                detector = detector.Detector(), load = True, type_input = "opencv")
    
    elif PERCEPTION_FUNCTION =="Openpifpaf":
        detection_image = classes.DetectorConfig(width = 161, height = 107, channels = 3, downscale = 3.58880,
                                                global_path = '',
                                                detector = pifpaf_detector.Detector_pifpaf(), load = False, type_input = "pil")


    # Initialize socket connections
    ip_address = rospy.get_param("/ip_address")
    socket1 = classes.SocketLoomo(8081, dt_perception/4, ip_address, detection_image.data_size)
    socket5 = classes.SocketLoomo(8085, dt_perception/4, ip_address, packer=25*'f ')
    # Perception visualization tools activated?
    visualization = False

    # Initialize perception variables
    net_received_length = 0
    received_image = b''

    rospy.loginfo("Perception Node Ready")

    while not rospy.is_shutdown():
        start = time.time()

        # Receive Image from the Loomo
        socket1.receiver(True)
        received_image += socket1.received_data
        net_received_length += len(socket1.received_data)

        # If detector and received image size are the same
        if net_received_length == socket1.data_size:
            # Detect object/human inside the image
            bbox_list, label_list = detection_image.detect(received_image)

            if visualization:
                plt.clf()
                plt.plot(0.0, 0.0, ">k")

            bbox = tuple()

            for i in range(len(bbox_list)):
                # Send bbox positions via socket to represent them in the Loomo
                bbox = bbox + (bbox_list[i][0], bbox_list[i][1], bbox_list[i][2], bbox_list[i][3], float(label_list[i][0]))
                
            bbox = bbox + (0.0,)*(25-len(bbox))
            socket5.sender(bbox)

            # Reset perception variables
            net_received_length = 0
            received_image = b''

        elif net_received_length > socket1.data_size:
            net_received_length = 0
            received_image = b''

        # Calculate node computation time
        computation_time = time.time() - start

        if computation_time > dt_perception:
            rospy.logwarn("Perception computation time higher than node period by " + str(computation_time-dt_perception) + " seconds")

        rate.sleep()


if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass