#!/usr/bin/env python
# VITA, EPFL
import rospy
from Estimation_Functions.IMU_data import *
from Estimation_Functions.SLAM import *
import matplotlib.pyplot as plt
import numpy as np
import time
from msg_types.msg import State, StateArray, TrajectoryArray, PositionArray


class Sender(object):
    def __init__(self):
        self.pub_states = rospy.Publisher('/Estimation/estimated_state', State, queue_size=1)

    def send(self, states):
        cmd_state = list2State(states)

        # Publish "/states" commands:
        self.pub_states.publish(cmd_state)


def main():
    # Initialize ROS
    rospy.init_node("estimation")
    dt_estimation = rospy.get_param("/estimation/dt_estimation")
    rate = rospy.Rate(int(1/dt_estimation))
    sender = Sender()

    # Initialize socket connections
    ip_address = rospy.get_param("/perception/ip_address")
    socket2 = SocketLoomo(8082, dt_estimation, ip_address, unpacker='f f f f f')

    # Estimation visualization tools activated?
    visualization = False

    # Parameter Initialization
    slam = SlamConfiguration(range_sensor=3.0, error_sensor=0.5)

    # Variable Initialization
    new_states = (0.0, 0.0, 0.0, 0.0, 0.0)
    initial_states = new_states
    i = 0

    rospy.loginfo("Estimation Node Ready")

    while not rospy.is_shutdown():
        start = time.time()

        # Receive IMU data from the Loomo
        socket2.receiver()

        if socket2.received_ok:
            states = socket2.received_data_unpacked

            if i <= 0:
                i = i + 1
                initial_states = states

            # Transform data in relation to the initial point
            new_states = state_with_initial_calibration(initial_states, states)

            if visualization:
                plt.clf()
                plt.plot(new_states[0], new_states[1], "ro")
                plt.arrow(new_states[0], new_states[1], np.cos(new_states[2]), np.sin(new_states[2]), width=0.1)
                plt.axis([-10, 10, -10, 10])
                plt.grid(True)
                plt.pause(0.001)

        # Send state estimation topics via ROS
        sender.send(new_states)

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