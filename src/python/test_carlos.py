# VITA, EPFL
import rospy
import cv2
import socket
import sys
import numpy
import struct
import math
from std_msgs.msg import Float64MultiArray
import select
import binascii

from PIL import Image
from detector import Detector
import argparse

def main():

    # Initialize ROS
    rospy.init_node("test_carlos")
    sub_imu = rospy.Subscriber('/pos_info', Float64MultiArray, callback_imu, queue_size = 1)
    rate = rospy.Rate(100)


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
    port1 = 8081
    port2 = 8082 # The port used by the server

    # image data
    downscale = args.downscale
    width = int(640/downscale)
    height = int(480/downscale)
    channels = 3
    sz_image = width*height*channels

    # Set up detector
    detector = Detector()
    detector.load('./saved_model.pth')

    # create socket
    print('# Creating socket')
    try:
        s1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
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
    s1.connect((remote_ip , port1))
    s2.connect((remote_ip , port2))

    #Image Receiver
    net_recvd_length = 0
    recvd_image = b''

    #Test Controller
    direction = -1
    cnt = 0
    states = []
    packer = struct.Struct('f f f f f')
    unpacker = struct.Struct('f f f f f')

    while not rospy.is_shutdown():

        cnt = cnt + 1
        # Receive data
        reply_image = s1.recv(sz_image)

        s2.setblocking(0)

        ready = select.select([s2], [], [], 0.005)



        if ready[0]:
            reply_states = s2.recv(unpacker.size)
            states = unpacker.unpack(reply_states)
            print(states)

        recvd_image += reply_image
        net_recvd_length += len(reply_image)




        if net_recvd_length == sz_image:
            print('Received Image Size: ' + str(net_recvd_length))
            print('Expected Image Size: ' + str(sz_image))
            pil_image = Image.frombytes('RGB', (width, height), recvd_image)
            opencvImage = cv2.cvtColor(numpy.array(pil_image), cv2.COLOR_RGB2BGR)
            opencvImage = cv2.cvtColor(opencvImage,cv2.COLOR_BGR2RGB)

            cv2.imshow('Test window',opencvImage)
            cv2.waitKey(1)

            net_recvd_length = 0
            recvd_image = b''

            bbox, bbox_label = detector.forward(opencvImage)

            if bbox_label:
                print(bbox)
                print(float(bbox_label[0]))
            else:
                print("False")

            # https://pymotw.com/3/socket/binary.html
            values = (bbox[0], bbox[1], bbox[2], bbox[3], float(bbox_label[0]))
            # https://pymotw.com/3/socket/binary.html
            #values = (40.0, 30.0, 10.0, 10.0, 0.0)

            packed_data = packer.pack(*values)

            # Send data
            send_info = s1.send(packed_data)

        elif net_recvd_length >= sz_image:
            net_recvd_length = 0
            recvd_image = b''

        rate.sleep()

def callback_imu(data):
    global x0_upc
    x0_upc = [data.x, data.y, data.heading, math.sqrt(data.vx**2 + data.vy**2)]
    print x0_upc


if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass