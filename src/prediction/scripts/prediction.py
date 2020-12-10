#!/usr/bin/env python
# VITA, EPFL
import rospy
from msg_types.msg import PositionArray, TrajectoryArray
from Prediction_Functions.linear_prediction import *
import matplotlib.pyplot as plt
import time

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath('/home/cconejob/StudioProjects/socket-loomo/src/perception/scripts/tools')))
from tools.classconverter import *
from tools.classes import *
from tools.transformations import *
from tools.utilities import *

class Sender(object):

    def __init__(self):
        self.pub_trajectories = rospy.Publisher('/Prediction/predicted_trajectories_global', TrajectoryArray, queue_size=1)

    def send(self, predicted_trajectories):
        predicted_trajectories_cmd = list2TrajectoryArray(predicted_trajectories)

        # Publish "/predicted_trajectory_array" commands:
        self.pub_trajectories.publish(predicted_trajectories_cmd)


def callback_estimation(data):

    global detections, start_prediction

    detections = PositionArray2list(data)
    start_prediction = True


def main():
    # Initialize ROS prediction node
    rospy.init_node("prediction")
    dt_prediction = rospy.get_param("/prediction/dt_prediction")
    rate = rospy.Rate(int(1/dt_prediction))
    prediction_activated = rospy.get_param("/perception/prediction_activated")
    mapping_activated = rospy.get_param("/perception/mapping_activated")

    if prediction_activated and mapping_activated:
        sub_estimation = rospy.Subscriber('/Estimation/map_global', PositionArray, callback_estimation, queue_size = 1)

    elif prediction_activated and not mapping_activated:
        ip_address = rospy.get_param("/perception/ip_address")
        socket3 = SocketLoomo(8083, dt_prediction, ip_address)

    dt_estimation = rospy.get_param("/estimation/dt_estimation")
    sender = Sender()

    # Prediction visualization tools activated?
    visualization = True

    # Initialize prediction parameters
    time_horizon_prediction = rospy.get_param("/prediction/time_horizon_prediction")
    past_number = rospy.get_param("/prediction/past_number")

    # Initialize prediction variables
    global detections, start_prediction

    detections = []
    past_detections = []
    start_prediction = False

    rospy.loginfo("Prediction Node Ready")
    rospy.sleep(2.)

    while not rospy.is_shutdown() and prediction_activated:
        start = time.time()

        if not mapping_activated:
            # Receive detection positions (x, y) in relation to the Loomo
            socket3.receiver()

            if socket3.received_ok:
                start_prediction = True
                positions = [socket3.received_data_unpacked]
                detections = []

                for idx,position in enumerate(positions):

                    if position[0]!=0.0:
                        detections.append(list(position)+[idx+1])

        if start_prediction:
            # Add present detections to past in a buffer
            past_detections, past_present_positions = add_detections_to_past(detections, past_detections, past_number)

            ################################ PREDICTION FUNCTION ###################################

            predicted_trajectories = linear_prediction(past_present_positions, dt_estimation, time_horizon_prediction)

            ########################################################################################

            # Plot past, actual and predicted positions
            if visualization:
                plt.clf()
                plt.plot(0.0, 0.0, ">k")

                for i, e1 in enumerate(predicted_trajectories):
                    plot_circle(past_present_positions[i][past_number-2][0], past_present_positions[i][past_number-2][1], 0.25,"r-")
                    cnt = 0

                    for e2 in e1:
                        if cnt==0:
                            plot_circle(e2[0],e2[1], 0.25,"b-")

                        else:
                            plot_circle(e2[0],e2[1], 0.25,"g--")

                        plt.axis([0, 10, -5, 5])
                        plt.grid(True)
                        cnt = cnt + 1

            # Send prediction topic via ROS
            sender.send(predicted_trajectories)

        # Calculate node computation time
        computation_time = time.time() - start
        
        if computation_time > dt_prediction:
            rospy.logwarn("Prediction computation time higher than node period by " + str(computation_time-dt_prediction) + " seconds")

        rate.sleep()


if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass