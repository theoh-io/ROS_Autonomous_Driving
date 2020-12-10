#!/usr/bin/env python
# VITA, EPFL
import rospy
import matplotlib.pyplot as plt
from Perception_Functions.detector import Detector
import time

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath('/home/cconejob/StudioProjects/socket-loomo/src/perception/scripts/tools')))
from tools.classes import *


def main():
    # Initialize ROS perception node
    rospy.init_node("perception")
    dt_perception = rospy.get_param("/perception/dt_perception")
    rate = rospy.Rate(int(1/dt_perception))

    # Initialize Detector Configuration
    # Set width, height and channel values for the received image --> Loomo image dimensions: 80x60x3
    # Set downscale as the relation between Loomo image/detector expected image
    detection_image = DetectorConfiguration(width = 80, height = 60, channels = 3, downscale = 1,
                                            global_path = '/home/cconejob/StudioProjects/socket-loomo/src/perception/scripts/Perception_Functions/saved_model.pth',
                                            detector = Detector())

    # Initialize socket connections
    ip_address = rospy.get_param("/perception/ip_address")
    socket1 = SocketLoomo(8081, dt_perception/2, ip_address, detection_image.data_size)

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

            for i in range(len(bbox_list)):
                # Send bbox positions via socket to represent them in the Loomo
                bbox = (bbox_list[i][0], bbox_list[i][1], bbox_list[i][2], bbox_list[i][3], float(label_list[i][0]))
                socket1.sender(bbox)

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