#!/usr/bin/env python3
# VITA, EPFL
import rospy
from Estimation_Functions import SLAM
import matplotlib.pyplot as plt
import time
from msg_types.msg import State, StateArray, TrajectoryArray, PositionArray
from tools import classconverter, classes
from tools.utils import Utils, Plotting


class Sender(object):
    def __init__(self):
        self.pub_map_global = rospy.Publisher('/State_Estimation/map_global', PositionArray, queue_size=1)
        self.pub_map_state = rospy.Publisher('/State_Estimation/map_state', StateArray, queue_size=1)

    def send(self, map_global, map_state=[]):

        global map_state_activated

        #print(f"map global {map_global}")
        cmd_map_global = classconverter.list2PositionArray(map_global)
        #print(f"cmd map global {cmd_map_global}")
        # Publish "/map" commands:
        self.pub_map_global.publish(cmd_map_global)

        if map_state_activated:
            #if map_state:
            cmd_map_state = classconverter.list2StateArray(map_state)
            print(f"cmd map state {cmd_map_state}")
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
    fake_obst = rospy.get_param("/fake_obst")
    verbose = rospy.get_param("/verbose_map")

    fake_obst_list=[[-3, 0.1], [2, 2]]
    
    if mapping_activated:
        sender = Sender()
        sub_estimation = rospy.Subscriber('/State_Estimation/estimated_state', State, callback_estimation, queue_size = 1)
        ip_address = rospy.get_param("/ip_address")
        socket3 = classes.SocketLoomo(8083, dt_mapping/4, ip_address, unpacker=10*'f ')

    # Parameter Initialization
    slam = SLAM.SlamConfiguration(range_sensor=10.0, error_sensor=1.0)

    # Variable Initialization
    global state, map_state_activated

    state = [0.0, 0.0, 0.0, 0.0, 0.0]
    map_state_activated = rospy.get_param("/map_state_activated")
    map_total = [] #contains positions of the obstacles

    map_state=[]
    list_positions=[]

    rospy.loginfo("Mapping Node Ready")

    rospy.sleep(0.2)

    #to initialize the timer
    data_received=True

    while not rospy.is_shutdown() and mapping_activated:
        if data_received is True:
            start = time.time()
            data_received=False
        # Receive detection positions (x, y) in relation to the Loomo
        socket3.receiver()

        # Add detections into a list
        if socket3.received_ok:
            data_received=True
            positions = [socket3.received_data_unpacked]
            #info sent by Extract Target on Loomo App: (depth and angle)
            if verbose: print(f"positions {positions}")
            list_positions=[]
            
            for idx in range(int(len(positions[0])/2)-1):
                #list positions has the relative positions of the detections plus index starting at 1
                if positions[0][idx*2]!=0.0:
                    list_positions.append([positions[0][idx*2], positions[0][idx*2+1], idx+1])
            if verbose:
                print(f"list_positions: {list_positions}")

            if fake_obst:
                if list_positions:
                    obst_number=len(list_positions)+1
                    for obst in fake_obst_list:
                        new_obst=obst+[obst_number]
                        list_positions.append(new_obst)
                        obst_number+=1
                else:
                    obst_number=1
                    for obst in fake_obst_list:
                        new_obst=obst+[obst_number]
                        if obst_number==1:
                            list_positions=[new_obst]
                        else:
                            list_positions.append(new_obst)
                        obst_number+=1
                        #print(list_positions)
            
            # Mapping function
            map_total, map_state = slam.mapping(state, list_positions)

            
                    

            if verbose:
                print(f"map total: {map_total}")#, map_state: {map_state}")
            #map_total=[[0.5, 0.5, 1]] #forcing map to create fake obstacles
            #print(f"new map total: {map_total}") 

        #Tentative handcoding Obstacles
        # else:
            
                    #map_total=[[-3, 0.1, 1], [1.0, -2, 2]] #forcing map to create fake obstacles
        #     #print("Hand coded obstacles !!!!!!")
        #     map_total=[[1.8, 0.1, 1]]


            
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
