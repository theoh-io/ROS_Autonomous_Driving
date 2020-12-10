# MICRO-453 TP-13
# Visual Navigation: A Deep Learning Perspective
# VITA, EPFL 
import __future__
import cv2
import socket
import sys
import numpy
import struct
import binascii

from PIL import Image
from detector import Detector

host = '128.179.188.218'  # The server's hostname or IP address

####################################
port = 8081       # The port used by the server

# image data
width = 80
height = 60
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

# Connect to remote server
print('# Connecting to server, ' + host + ' (' + remote_ip + ')')
s.connect((remote_ip , port))

# Set up detector
detector = Detector()
detector.load('./saved_model.pth')

img = cv2.imread('./minion3.jpg', -1)

img = cv2.resize(img,(width, height))

bbox, bbox_label = detector.forward(img)
print(float(bbox_label[0]))

#Image Receiver 
net_recvd_length = 0
recvd_image = b''

#Test Controller
direction = -1
cnt = 0

while True:

    # Receive data
    reply = s.recv(sz_image)
    #print(reply)
    recvd_image += reply

    net_recvd_length += len(reply)


    if net_recvd_length >= sz_image:
        #print('# Image being printed')
        #print('Received Image Size: ' + str(net_recvd_length))
        #print('Expected Image Size: ' + str(sz_image))

        pil_image = Image.frombytes('RGB', (width, height), recvd_image)
        opencvImage = cv2.cvtColor(numpy.array(pil_image), cv2.COLOR_RGB2BGR)
        opencvImage = cv2.cvtColor(opencvImage,cv2.COLOR_BGR2RGB)

        cv2.imshow('Test window',opencvImage)
        cv2.waitKey(1)

        net_recvd_length = 0
        recvd_image = b''

        #######################
        # Detect
        #######################
        bbox, bbox_label = detector.forward(opencvImage)
        
        #if bbox_label:
            #print(bbox)
        #else:
            #print("False")

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

    #else:
        #print('# Image not being printed')
