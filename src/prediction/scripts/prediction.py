#!/usr/bin/env python3
# VITA, EPFL
import rospy
from msg_types.msg import PositionArray, TrajectoryArray
from Prediction_Functions import linear_prediction, trajnetplus_predictor
import matplotlib.pyplot as plt
import time

import os
import sys
abs_path_to_tools = rospy.get_param("/abs_path_to_tools")
sys.path.append(os.path.dirname(os.path.abspath(abs_path_to_tools)))
from tools import classconverter, classes, transformations
from tools.utils import Utils, Plotting

import rospkg


class Sender(object):

    def __init__(self):
        self.pub_trajectories = rospy.Publisher('/Prediction/predicted_trajectories_global', TrajectoryArray, queue_size=1)

    def send(self, predicted_trajectories):
        predicted_trajectories_cmd = classconverter.list2TrajectoryArray(predicted_trajectories)

        # Publish "/predicted_trajectory_array" commands:
        self.pub_trajectories.publish(predicted_trajectories_cmd)


def callback_estimation(data):

    global map_detections

    map_detections = classconverter.PositionArray2list(data)


def main():
    # Initialize ROS prediction node
    rospy.init_node("prediction")
    dt_prediction = rospy.get_param("/dt_prediction")
    rate = rospy.Rate(int(1/dt_prediction))
    prediction_activated = rospy.get_param("/prediction_activated")
    mapping_activated = rospy.get_param("/mapping_activated")
    PREDICTION_FUNCTION = rospy.get_param("/PREDICTION_FUNCTION")
    #path_model = rospy.get_param("/model_prediction_path")
    rospack=rospkg.RosPack()
    abs_path_to_prediction=rospack.get_path('prediction')
    path_model=os.path.abspath(os.path.join(abs_path_to_prediction,"scripts/Prediction_Functions/models/lstm_directional_one_12_6_rerun.pkl"))
    print(f"prediction path")

    if prediction_activated and not mapping_activated:
        ip_address = rospy.get_param("/ip_address")
        socket4 = classes.SocketLoomo(8084, dt_prediction/4, ip_address, unpacker=10*'f ')

    elif prediction_activated and mapping_activated:
        sub_estimation = rospy.Subscriber('/State_Estimation/map_global', PositionArray, callback_estimation, queue_size = 1)

    sender = Sender()

    # Initialize prediction parameters
    time_horizon_prediction = rospy.get_param("/time_horizon_prediction")
    n_past_observations = rospy.get_param("/past_observations")

    if PREDICTION_FUNCTION == "Default":
        predictor = linear_prediction.LinealPredictor(dt=dt_prediction, pred_horizon=time_horizon_prediction, obs_length=n_past_observations)
    
    elif PREDICTION_FUNCTION == "Trajnet":
        predictor = trajnetplus_predictor.TrajNetPredictor(dt=dt_prediction, pred_horizon=time_horizon_prediction, obs_length=n_past_observations, model=path_model)

    # Initialize prediction variables
    global map_detections

    detections = []
    map_detections = []
    past_detections = []
    positions = [[0.0,0.0]]

    #Print to help debugging
    verbose=True

    rospy.loginfo("Prediction Node Ready")
    rospy.sleep(3.)

    while not rospy.is_shutdown() and prediction_activated:
        start = time.time()
        detections = map_detections

        # Receive detection positions (x, y) in relation to the Loomo
        if not mapping_activated:
            socket4.receiver()      

            if socket4.received_ok:
                positions = [socket4.received_data_unpacked]
                detections = []

            if verbose:
                print(f"in prediction: positions {positions}")

            for idx in range(int(len(positions[0])/2)-1):
                
                if positions[0][idx*2]!=0.0:
                    detections.append([positions[0][idx*2], positions[0][idx*2+1], idx+1])
        
        else:
            if verbose:
                print(f"in prediction:  {detections}")


        # Add present detections to past in a buffer and predict trajectories
        past_detections, past_present_positions = Utils.add_detections_to_past(detections, past_detections, n_past_observations)
        predicted_trajectories = predictor.prediction_function(past_present_positions)

        if verbose:
                print(f"in prediction: pred_traj {predicted_trajectories}")

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