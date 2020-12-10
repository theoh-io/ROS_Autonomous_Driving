#!/usr/bin/env python
# VITA, EPFL
import rospy
from msg_types.msg import Position, PositionArray, TrajectoryArray, State, StateArray
from Path_planning_Functions.RRT_star import *
import time


class Sender(object):

    def __init__(self):
        self.pub_array_path_local = rospy.Publisher('/Path_planning/desired_states_local', StateArray, queue_size=1)


    def send(self, desired_path, objects_now, x0):
        array_path_local = list2StateArray(desired_path)
        array_objects = list2TrajectoryArray(objects_now)
        state_planner = list2State(x0)
        array_path_local.sync_predictions = array_objects
        array_path_local.initial_state = state_planner

        # Publish "/desired_trajectory" commands:
        self.pub_array_path_local.publish(array_path_local)


def callback_prediction(data): # data is TrajectoryArray --> [[[x_11,y_11],[x_12,y_12]],[[x_21,y_21],[x_22,y_22]]]

    global array_predictions

    array_predictions = TrajectoryArray2list(data)


def callback_estimation(data): # data is State --> [x, y, heading]

    global state

    state = State2list(data)


def callback_mapping(data):

    global array_predictions

    array_predictions = PositionArray2TrajectoryArraylist(data)
    print(array_predictions)




def main():
    # Initialize ROS
    rospy.init_node("path_planning")
    dt_path_planning = rospy.get_param("/path_planning/dt_path_planning")
    rate = rospy.Rate(int(1/dt_path_planning))
    sub_estimation = rospy.Subscriber('/Estimation/estimated_state', State, callback_estimation, queue_size = 1)
    prediction_activated = rospy.get_param("/perception/prediction_activated")
    mapping_activated = rospy.get_param("/perception/mapping_activated")
    direct_path_planning = False

    if prediction_activated:
        sub_prediction = rospy.Subscriber('/Prediction/predicted_trajectories_global', TrajectoryArray, callback_prediction, queue_size = 1)
        
    elif mapping_activated:
        sub_mapping = rospy.Subscriber('/Estimation/map_global', PositionArray, callback_mapping, queue_size = 1)

    else:
        ip_address = rospy.get_param("/perception/ip_address")
        socket3 = SocketLoomo(8083, dt_path_planning/4, ip_address)
        direct_path_planning = True

    sender = Sender()

    # Initialize path planning parameters
    dt_control = rospy.get_param("/control/dt_control")
    speed = rospy.get_param("/path_planning/speed")
    wheel_base = rospy.get_param("/control/wheel_base") # m
    v_max = rospy.get_param("/control/v_max") # m/s
    loomo = MobileRobot(wheel_base, v_max)

    # Initialize path planning variables
    global array_predictions, state

    previous_array_predictions = []
    array_predictions = []
    goal_global = [10.0, 5.0]
    state = [0.0, 0.0, 0.0]
    objects_now = []
    cnt = 1

    rospy.loginfo("Path Planning Node Ready")
    rospy.sleep(2.)

    while not rospy.is_shutdown():
        cnt = cnt - 1

        if direct_path_planning:
            # Receive detection positions (x, y) in relation to the Loomo
            socket3.receiver()

            if socket3.received_ok:
                start_prediction = True
                positions = [socket3.received_data_unpacked]
                array_predictions = []

                for idx,position in enumerate(positions):

                    if position[0]!=0.0:
                        array_predictions.append([list(position)+[idx+1]+[0.0]])

        if cnt <= 0:
            start = time.time()

            x0 = state
            goal_local = Global_to_Local(x0, [goal_global], True)[0]

            # Actual detection movement prediction
            if len(array_predictions) > 0:
                objects_now = Global_to_Local_prediction(x0, array_predictions)

            # Obstacle avoidance path calculation

            ############################ PATH PLANNING FUNCTION ####################################

            path = planner_rrt_star(loomo, objects_now, speed, dt_control, goal_local, prediction_activated=True)

            ########################################################################################

            if len(path)>5:
                # Send commands to the ROS Structure
                sender.send(path, objects_now, x0)
                cnt = 1

            # Calculate node computation time
            computation_time = time.time() - start

            if computation_time > dt_path_planning:
                rospy.logwarn("Path planning computation time higher than node period by " + str(computation_time-dt_path_planning) + " seconds")

            objects_now = []

        rate.sleep()


if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass
