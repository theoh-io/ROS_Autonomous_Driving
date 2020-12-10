#!/usr/bin/env python
# VITA, EPFL
import rospy
from msg_types.msg import Position, PositionArray, TrajectoryArray, State, StateArray, ControlCmd
from Control_Functions.MPC_Control import *
from Control_Functions.MPC_Eloi import *
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
        self.pub_control = rospy.Publisher('/Control/control_commands', ControlCmd, queue_size=1)
        self.visualization_activated = rospy.get_param("/perception/visualization_activated")

        if self.visualization_activated:
            self.pub_predicted_states_local = rospy.Publisher('/Visualization/predicted_states_local', StateArray, queue_size=1)
            self.pub_actual_path_planning_global = rospy.Publisher('/Visualization/desired_states_local', StateArray, queue_size=1)
            self.pub_actual_state = rospy.Publisher('/Visualization/estimated_state', State, queue_size=1)
            self.pub_planner_state = rospy.Publisher('/Visualization/planner_state', State, queue_size=1)
            self.pub_actual_predictions_local = rospy.Publisher('/Visualization/predictions_local', TrajectoryArray, queue_size=1)

    def send(self, control_cmd, predicted_states_l, actual_state, state_planner, path_l, predictions_l):
        control_command = list2ControlCmd(control_cmd)

        if self.visualization_activated:
            predicted_states_local = list2StateArray(predicted_states_l)
            path_global = list2StateArray(path_l)
            actual_state = list2State(actual_state)
            planner_state = list2State(state_planner)
            actual_predictions = list2TrajectoryArray(predictions_l)

        # Publish commands:
        self.pub_control.publish(control_command)

        if self.visualization_activated:
            self.pub_actual_state.publish(actual_state)
            self.pub_planner_state.publish(planner_state)
            self.pub_predicted_states_local.publish(predicted_states_local)
            self.pub_actual_path_planning_global.publish(path_global)
            self.pub_actual_predictions_local.publish(actual_predictions)


def callback_path_planning_l(data): # data is StateArray --> [[x_1, y_1, heading_1, v_1, w_1], [x_2, y_2, heading_2, v_2, w_2], ...]

    global local_path, x0, local_predictions

    local_path = StateArray2list(data)
    local_predictions = TrajectoryArray2list(data.sync_predictions)
    x0 = State2list(data.initial_state)

    global global_path

    global_path = Local_to_Global(x0,local_path)


def callback_estimation(data): # data is State --> [x, y, heading]

    global state

    state = State2list(data)


def main():
    # Initialize ROS control node
    rospy.init_node("control")
    dt_control = rospy.get_param("/control/dt_control")
    rate = rospy.Rate(int(1/dt_control))
    sub_path_planning_global = rospy.Subscriber('/Path_planning/desired_states_local', StateArray, callback_path_planning_l, queue_size = 1)
    sub_estimation = rospy.Subscriber('/Estimation/estimated_state', State, callback_estimation, queue_size = 1)
    sender = Sender()

    # Initialize socket connection
    ip_address = rospy.get_param("/perception/ip_address")
    socket0 = SocketLoomo(8080, dt_control, ip_address, packer="f f")

    # Initialize control parameters
    wheel_base = rospy.get_param("/control/wheel_base") # m
    v_max = rospy.get_param("/control/v_max") # m/s
    loomo = MobileRobot(wheel_base, v_max)
    control_prediction_horizon = rospy.get_param("/control/control_prediction_horizon") # s
    #mpc = MPC(loomo, dt_control, control_prediction_horizon)
    mpc = MPC_model()
    n_states = rospy.get_param("/control/n_states")

    # Initialize control variables
    global local_path, state, x0, planner_counter, local_predictions
    #global global_path
    actual_state = [0.0] * n_states
    actual_path = []
    state = [0.0] * n_states
    state_planner = state
    local_path = []
    #global_path = []
    planner_counter = 0
    predicted_states_local = []
    local_predictions = []

    rospy.loginfo("Control Node Ready")
    rospy.sleep(5.)

    while not rospy.is_shutdown():
        start = time.time()
        control_cmd = [0.0, 0.0]

        if len(local_path) > int(mpc.HORIZON_N):
            # Initial planner state and Actual state in global coordinates
            state_planner = x0
            actual_state = state
            # Actual state in relation to first path planning position
            state_local = Global_to_Local(state_planner, [actual_state])[0]

            # Desired states we need for the control algorithm
            planner_counter_l = minimum_distance(state_local, local_path)
            planner_counter_g = minimum_distance(actual_state, global_path)
            actual_path_local = local_path[planner_counter_l:]
            actual_path_global = global_path[planner_counter_g:]
            desired_path_local = actual_path_local[:mpc.HORIZON_N+1]
            desired_path_global = actual_path_global[:mpc.HORIZON_N+1]
            mpc.acquire_path(desired_path_global)
            local_predictions = local_predictions

            # Calculate the actual control command and the control path it predicts to follow.
            if len(desired_path_global)>int(mpc.HORIZON_N):
                #control_cmd, predicted_states_local = mpc_control_loomo(mpc, state_local, desired_path_local)
                control_cmd, predicted_states_global = mpc.run_MPC(actual_state)
                predicted_states_local = Global_to_Local(actual_state, predicted_states_global)
                rospy.loginfo(predicted_states_global)
                rospy.loginfo(len(predicted_states_global))

            else:
                control_cmd = [0.0, 0.0]
                rospy.logwarn("OBJECT REACHED FINAL POSITION")

        else:
            rospy.logerr("Not enough planner length. Stopping vehicle...")

        # Send control commands + visualization topics via ROS
        sender.send(control_cmd, predicted_states_local, actual_state, state_planner, actual_path_local, local_predictions)

        # Send control commands via socket
        values = (control_cmd[0], control_cmd[1])
        socket0.sender(values)

        # Calculate node computation time
        computation_time = time.time() - start
        
        if computation_time > dt_control:
            rospy.logwarn("Control computation time higher than node period by " + str(computation_time-dt_control) + " seconds")

        rate.sleep()


if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass