#!/usr/bin/env python3
# VITA, EPFL
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import socket
import numpy as np
import struct
import select
from PIL import Image
import rospy


class SocketLoomo:
    # Initialize socket connection
    def __init__(self, port, dt, host, data_size = 0, packer = 25*'f ', unpacker = 10*'f ', sockettype = "Stream"):
        self.data_size = data_size
        self.max_waiting_time = dt/10
        self.received_data = []
        self.received_ok = False
        self.received_data_unpacked = []

        try:
            if sockettype == "Stream":
                self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
            else:
                self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        except socket.error:
            rospy.logerr('Failed to create socket')
            sys.exit()

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
                #print(self.received_data)

    # Send data to the Loomo
    def sender(self, values):
        # Pack data from floats to bytes
        packed_data = self.packer.pack(*values)

        self.s.send(packed_data)


class DetectorConfig:
    # Initialize detector and its main properties
    def __init__(self, width, height, channels, downscale, global_path='', detector='', load=True, type_input="opencv", save_video=False, filename_video=""):
        # Detector expected input image dimensions
        self.width = int(width)
        self.height = int(height)
        self.downscale = downscale

        if self.downscale == 1:
            self.scale_necessary = False

        else:
            self.scale_necessary = True

        # Image received size data.
        self.data_size = int(width * height * channels/downscale)
        self.global_path = global_path
        self.detector = detector
        self.type_input = type_input

        if load:
            self.detector.load(global_path)

        if save_video:
            self.save_video = save_video
            self.result = cv2.VideoWriter(filename_video, cv2.VideoWriter_fourcc(*'MJPG'), 10, (self.width, self.height))
    

    def detect(self, received_image):
        # Adapt image to detector requirements
        pil_image = Image.frombytes('RGB', (160,120), received_image)

        if self.scale_necessary:
            maxsize = (self.width, self.height)
            pil_image = pil_image.resize(maxsize, Image.ANTIALIAS)

        opencvImage = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
        opencvImage = cv2.cvtColor(opencvImage,cv2.COLOR_BGR2RGB)

        # Uncomment if you want to see the same as the robot from the computer. 
        
        #cv2.imshow('Test window',opencvImage)
        #cv2.waitKey(1)

        if self.save_video:
            self.result.write(opencvImage)

        if self.type_input == "opencv":
            image = opencvImage
        
        elif self.type_input == "pil":
            image = pil_image

        bbox_list, bbox_label, bbox_legs = self.detector.forward(image, self.downscale)

        return bbox_list, bbox_label, bbox_legs


class MobileRobot:

    def __init__(self, wheel_base, v_max):
        self.wheel_base = wheel_base
        self.v_max = v_max
        self.w_max = v_max / wheel_base

