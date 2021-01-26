#!/usr/bin/env python
# VITA, EPFL
import rospy
from Estimation_Functions import SLAM
import matplotlib.pyplot as plt
import time
from msg_types.msg import State, StateArray, TrajectoryArray, PositionArray
from tools import classconverter, classes


class Sender(object):
    def __init__(self):
        self.pub_map_global = rospy.Publisher('/State_Estimation/map_global', PositionArray, queue_size=1)
        self.pub_map_state = rospy.Publisher('/State_Estimation/map_state', State, queue_size=1)

    def send(self, map_global, map_state=[]):

        global map_state_activated

        cmd_map_global = classconverter.list2PositionArray(map_global)
        # Publish "/map" commands:
        self.pub_map_global.publish(cmd_map_global)

        if map_state_activated:
            cmd_map_state = classconverter.list2State(map_state)
            self.pub_map_state.publish(cmd_map_state)


def callback_estimation(data): # data is State --> [x, y, heading]

    global state

    state = classconverter.State2list(data)


def main():
    # Initialize ROS
    rospy.init_node("map_state")
    dt_mapping = rospy.get_param("/dt_map_state")
    rate = rospy.Rate(int(1/dt_mapping))
    mapping_activated = rospy.get_param("/mapping_activated")
    
    if mapping_activated:
        sender = Sender()
        sub_estimation = rospy.Subscriber('/State_Estimation/estimated_state', State, callback_estimation, queue_size = 1)
        ip_address = rospy.get_param("/ip_address")
        socket3 = classes.SocketLoomo(8083, dt_mapping/4, ip_address, unpacker=10*'f ')

    # Parameter Initialization
    slam = SLAM.SlamConfiguration(range_sensor=5.0, error_sensor=1.0)

    # Variable Initialization
    global state, map_state_activated

    state = [0.0, 0.0, 0.0, 0.0, 0.0]
    map_state_activated = rospy.get_param("/map_state_activated")
    map_total = []

    rospy.loginfo("Mapping Node Ready")

    rospy.sleep(1.5)

    while not rospy.is_shutdown() and mapping_activated:
        start = time.time()

        # Receive detection positions (x, y) in relation to the Loomo
        socket3.receiver()

        # Add detections into a list
        if socket3.received_ok:
            positions = [socket3.received_data_unpacked]
            list_positions = []

            for idx in range(int(len(positions[0])/2)-1):
                
                if positions[0][idx*2]!=0.0:
                    list_positions.append([positions[0][idx*2], positions[0][idx*2+1], idx+1])
    
            # Mapping function
            map_total, map_state = slam.mapping(state, list_positions)
            map_total = [[0.75,0.3,1],[1.3,-0.4,2],[2.2, -0.4,3]]
        # Send state estimation topics via ROS
        if map_state_activated:
            sender.send(map_total, map_state)

        else:
            sender.send(map_total)

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