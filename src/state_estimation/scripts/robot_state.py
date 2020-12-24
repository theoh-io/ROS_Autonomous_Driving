#!/usr/bin/env python
# VITA, EPFL
import rospy
from Estimation_Functions import IMU_data, motion_estimation_robot
import matplotlib.pyplot as plt
import numpy as np
import time
from msg_types.msg import State, StateArray, TrajectoryArray, PositionArray
from tools import classconverter, classes


class Sender(object):
    def __init__(self):
        self.pub_states = rospy.Publisher('/State_Estimation/estimated_state', State, queue_size=1)
        self.pub_gt = rospy.Publisher('/State_Estimation/ground_truth', State, queue_size=1)

    def send(self, new_states, states=[]):
        cmd_state = classconverter.list2State(new_states)

        # Publish "/states" commands:
        self.pub_states.publish(cmd_state)

        if len(states) > 0:
            cmd_gt = classconverter.list2State(states)
            self.pub_gt.publish(cmd_gt)


def main():
    # Initialize ROS
    rospy.init_node("robot_state")
    dt_estimation = rospy.get_param("/dt_robot_state")
    rate = rospy.Rate(int(1/dt_estimation))
    sender = Sender()

    # Initialize socket connections
    ip_address = rospy.get_param("/ip_address")
    socket2 = classes.SocketLoomo(8082, dt_estimation/4, ip_address, unpacker=5*'f ')

    # Estimation visualization tools activated?
    visualization = False

    # Parameter Initialization
    ESTIMATION_FUNCTION = rospy.get_param("/ROBOT_STATE_FUNCTION")

    if ESTIMATION_FUNCTION == "EPFL_Driverless":
        estimator = motion_estimation_robot.Driverless_Estimation()

    # Variable Initialization
    new_states = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    past_states = new_states
    initial_states = new_states
    states = new_states
    i = 0

    rospy.loginfo("Robot_State Node Ready")

    rospy.sleep(3.)

    while not rospy.is_shutdown():
        start = time.time()

        # Receive IMU data from the Loomo
        socket2.receiver()

        if socket2.received_ok:
            states = list(socket2.received_data_unpacked)
            #rospy.loginfo("(ax, ay):  (" + str(states[5])+ ", " + str(states[6]) + ")")
            a = (states[3]-past_states[3])/dt_estimation
            states.append(a)
            past_states = states
            if i <= 0:
                i = i + 1
                initial_states = states

            # Transform data in relation to the initial point
            if ESTIMATION_FUNCTION == "Default":
                new_states = IMU_data.state_with_initial_calibration(initial_states, states)
                # Send state estimation topics via ROS
                sender.send(new_states)

            elif ESTIMATION_FUNCTION == "EPFL_Driverless":
                states = IMU_data.state_with_initial_calibration(initial_states, states)
                new_states = estimator.kalman(states)
                # Send state estimation topics via ROS
                sender.send(new_states, states)

        # Calculate node computation time
        computation_time = time.time() - start
        
        if computation_time > dt_estimation:
            rospy.logwarn("Estimation computation time higher than node period by " + str(computation_time-dt_estimation) + " seconds")

        rate.sleep()


if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass