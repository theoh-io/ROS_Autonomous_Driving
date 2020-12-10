#!/usr/bin/env python
# VITA, EPFL
import rospy
from Estimation_Functions.SLAM import *
import matplotlib.pyplot as plt
import time
from msg_types.msg import State, StateArray, TrajectoryArray, PositionArray


class Sender(object):
    def __init__(self):
        self.pub_map_global = rospy.Publisher('/Estimation/map_global', PositionArray, queue_size=1)

    def send(self, map_global):
        cmd_map_global = list2PositionArray(map_global)

        # Publish "/map" commands:
        self.pub_map_global.publish(cmd_map_global)


def callback_estimation(data): # data is State --> [x, y, heading]

    global state

    state = State2list(data)


def main():
    # Initialize ROS
    rospy.init_node("mapping")
    dt_mapping = rospy.get_param("/mapping/dt_mapping")
    rate = rospy.Rate(int(1/dt_mapping))
    mapping_activated = rospy.get_param("perception/mapping_activated")
    
    if mapping_activated:
        sender = Sender()
        sub_estimation = rospy.Subscriber('/Estimation/estimated_state', State, callback_estimation, queue_size = 1)

    # Initialize socket connections
    if mapping_activated:
        ip_address = rospy.get_param("/perception/ip_address")
        socket3 = SocketLoomo(8083, dt_mapping, ip_address)

    # Estimation visualization tools activated?
    visualization = False

    # Parameter Initialization
    slam = SlamConfiguration(range_sensor=3.0, error_sensor=0.5)

    # Variable Initialization
    global state

    state = [0.0, 0.0, 0.0, 0.0, 0.0]

    rospy.loginfo("Mapping Node Ready")

    while not rospy.is_shutdown() and mapping_activated:
        start = time.time()

        # Receive detection positions (x, y) in relation to the Loomo
        socket3.receiver()

        if socket3.received_ok:
            positions = [socket3.received_data_unpacked]
            list_positions = []

            for position in positions:

                if position[0]!=0.0:
                    list_positions.append(list(position))

                    if visualization:
                        plot_circle(position[0],position[1], 0.25,"b-")
                        plt.axis([0, 10, -5, 5])
                        plt.pause(0.001)
                        plt.grid(True)

            ################################## MAPPING FUNCTION ####################################

            mapping(slam, state, list_positions)

            ########################################################################################

        # Send state estimation topics via ROS
        sender.send(slam.saved_map)

        # Calculate node computation time
        computation_time = time.time() - start
        
        if computation_time > dt_mapping:
            rospy.logwarn("Mapping computation time higher than node period by " + str(computation_time-dt_mapping) + " seconds")

        rate.sleep()


if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass