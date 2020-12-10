# VITA, EPFL

import cv2
import socket
import sys
import numpy
import struct
import binascii
import os

from PIL import Image
from detector import Detector
import argparse

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
)

parser.add_argument('-i','--ip-address',
                    help='IP Address of robot')
parser.add_argument('-d', '--downscale', default=8, type=int,
                    help=('downscale of the received image'))
args = parser.parse_args()

##### IP Address of server #########
host = args.ip_address #'128.179.150.43'  # The server's hostname or IP address
####################################
port = 8081        # The port used by the server

# image data
downscale = args.downscale
width = int(640/downscale)
height = int(480/downscale)
channels = 3
sz_image = width*height*channels

# create socket
print('# Creating socket')
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
except socket.error:
    print('Failed to create socket')
    sys.exit()

print('# Getting remote IP address')
try:
    remote_ip = socket.gethostbyname( host )
except socket.gaierror:
    print('Hostname could not be resolved. Exiting')
    sys.exit()

# Set up detector
detector = Detector()
value = os.path.exists('./saved_model.pth')
print(value)
detector.load('./saved_model.pth')

# Connect to remote server
print('# Connecting to server, ' + host + ' (' + remote_ip + ')')
s.connect((remote_ip , port))

#Image Receiver
net_recvd_length = 0
recvd_image = b''

#Test Controller
direction = -1
cnt = 0

while True:

    # Receive data
    reply = s.recv(sz_image)
    recvd_image += reply
    net_recvd_length += len(reply)

    if net_recvd_length == sz_image:

        pil_image = Image.frombytes('RGB', (width, height), recvd_image)
        opencvImage = cv2.cvtColor(numpy.array(pil_image), cv2.COLOR_RGB2BGR)
        opencvImage = cv2.cvtColor(opencvImage,cv2.COLOR_BGR2RGB)

        cv2.imshow('Test window',opencvImage)
        cv2.waitKey(1)

        net_recvd_length = 0
        recvd_image = b''

        # https://pymotw.com/3/socket/binary.html
        values = (40.0, 30.0, 10.0, 10.0, 0.0)

        packer = struct.Struct('f f f f f')
        packed_data = packer.pack(*values)

        # Send data
        send_info = s.send(packed_data)

        #######################
        # Detect
        #######################
        bbox, bbox_label = detector.forward(opencvImage)

        if bbox_label:
            print(bbox)
        else:
            print("False")

        # https://pymotw.com/3/socket/binary.html
        values = (bbox[0], bbox[1], bbox[2], bbox[3], float(bbox_label[0]))
        # values = (50.0, 30.0, 10.0, 10.0, 1.0)

        # #Test Controller
        # cnt = cnt + 1
        # if cnt > 50:
        #     direction = - direction
        #     cnt = 0
        # values = (40.0 + direction * 20.0, 30.0, 10.0, 20.0, 1.0)

        packer = struct.Struct('f f f f f')
        packed_data = packer.pack(*values)

        # Send data
        send_info = s.send(packed_data)

    else:
        print('# Image not being printed')