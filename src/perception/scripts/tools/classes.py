#!/usr/bin/env python
# VITA, EPFL
import cv2
import socket
import sys
import numpy as np
import struct
import select
from PIL import Image
import rospy


class SocketLoomo:
    # Initialize socket connection
    def __init__(self, port, dt, host, data_size = 0, packer = 'f f f f f', unpacker = 'f f'):
        self.data_size = data_size
        self.max_waiting_time = dt/10
        self.received_data = []
        self.received_ok = False
        self.received_data_unpacked = []
        rospy.loginfo('# Creating perception socket')

        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        except socket.error:
            rospy.logerr('Failed to create perception socket')
            sys.exit()

        rospy.loginfo('# Getting remote IP address')

        try:
            remote_ip = socket.gethostbyname(host)

        except socket.gaierror:
            rospy.logerr('Hostname could not be resolved. Exiting')
            sys.exit()

        rospy.loginfo('# Connecting to server, ' + host + ' (' + str(port) + ')')
        
        # Connect to IP address and port
        self.s.connect((remote_ip , port))
        # Create the packer
        self.packer = struct.Struct(packer)
        # Create the unpacker
        self.unpacker = struct.Struct(unpacker)

    # Receive data from the Loomo
    def receiver(self, is_image = False):

        if is_image:
            self.received_data = b''

        self.s.setblocking(0)
        # Set a timout value
        ready = select.select([self.s], [], [], self.max_waiting_time)
        self.received_ok = False

        # If data received before timeout value
        if ready[0]:
            self.received_ok = True

            if not is_image:
                self.received_data = self.s.recv(self.unpacker.size)
                # Unpack data converting it from bytes to floats
                self.received_data_unpacked = self.unpacker.unpack(self.received_data)

            else:
                self.received_data = self.s.recv(self.data_size)

    # Send data to the Loomo
    def sender(self, values):
        # Pack data from floats to bytes
        packed_data = self.packer.pack(*values)

        self.s.send(packed_data)


class DetectorConfiguration:
    # Initialize detector and its main properties
    def __init__(self, width, height, channels, downscale, global_path, detector):
        # Detector expected input image dimensions
        self.width = int(width/downscale)
        self.height = int(height/downscale)
        # Image received size data.
        self.data_size = width * height * channels
        self.global_path = global_path
        self.detector = detector
        self.detector.load(global_path)

    def detect(self, received_image):
        # Adapt image to detector requirements
        pil_image = Image.frombytes('RGB', (self.width, self.height), received_image)
        opencvImage = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
        opencvImage = cv2.cvtColor(opencvImage,cv2.COLOR_BGR2RGB)
        cv2.imshow('Test window',opencvImage)
        cv2.waitKey(1)
        bbox_list, bbox_label = self.detector.forward(opencvImage)

        return bbox_list, bbox_label


class MobileRobot:

    def __init__(self, wheel_base, v_max):
        self.wheel_base = wheel_base
        self.v_max = v_max
        self.w_max = v_max / wheel_base