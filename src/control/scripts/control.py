#!/usr/bin/env python
# VITA, EPFL
import rospy
from msg_types.msg import Position, PositionArray, TrajectoryArray, State, StateArray, ControlCmd
from Control_Functions import MPC_Control, MPC_Eloi
import time

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath('/home/cconejob/StudioProjects/socket-loomo/src/perception/scripts/tools')))
from tools import classconverter, classes, transformations, utilities

class Sender(object):

    def __init__(self):
        self.pub_control = rospy.Publisher('/Control/control_commands', ControlCmd, queue_size=1)
        self.visualization_activated = rospy.get_param("/visualization_activated")

        if self.visualization_activated:
            self.pub_predicted_states_local = rospy.Publisher('/Visualization/predicted_states_local', StateArray, queue_size=1)
            self.pub_actual_path_planning_global = rospy.Publisher('/Visualization/desired_states_local', StateArray, queue_size=1)
            self.pub_actual_state = rospy.Publisher('/Visualization/estimated_state', State, queue_size=1)
            self.pub_planner_state = rospy.Publisher('/Visualization/planner_state', State, queue_size=1)
            self.pub_actual_predictions_local = rospy.Publisher('/Visualization/predictions_local', TrajectoryArray, queue_size=1)

    def send(self, control_cmd, predicted_states_l, actual_state, state_planner, path_l, predictions_l):
        control_command = classconverter.list2ControlCmd(control_cmd)

        if self.visualization_activated:
            predicted_states_local = classconverter.list2StateArray(predicted_states_l)
            path_global = classconverter.list2StateArray(path_l)
            actual_state = classconverter.list2State(actual_state)
            planner_state = classconverter.list2State(state_planner)
            actual_predictions = classconverter.list2TrajectoryArray(predictions_l)

        # Publish commands:
        self.pub_control.publish(control_command)

        if self.visualization_activated:
            self.pub_actual_state.publish(actual_state)
            self.pub_planner_state.publish(planner_state)
            self.pub_predicted_states_local.publish(predicted_states_local)
            self.pub_actual_path_planning_global.publish(path_global)
            self.pub_actual_predictions_local.publish(actual_predictions)


def callback_path_planning_l(data): # data is StateArray --> [[x_1, y_1, heading_1, v_1, w_1], [x_2, y_2, heading_2, v_2, w_2], ...]

    global local_path, x0, local_predictions, global_path

    local_path = classconverter.StateArray2list(data)
    local_predictions = classconverter.TrajectoryArray2list(data.sync_predictions)
    x0 = classconverter.State2list(data.initial_state)
    global_path = transformations.Local_to_Global(x0,local_path)


def callback_estimation(data): # data is State --> [x, y, heading]

    global state

    state = classconverter.State2list(data)


def main():
    # Initialize ROS control node
    rospy.init_node("control")
    dt_control = rospy.get_param("/dt_control")
    rate = rospy.Rate(int(1/dt_control))
    sub_path_planning_global = rospy.Subscriber('/Path_planning/desired_states_local', StateArray, callback_path_planning_l, queue_size = 1)
    sub_estimation = rospy.Subscriber('/State_Estimation/estimated_state', State, callback_estimation, queue_size = 1)
    sender = Sender()

    # Initialize socket connection
    ip_address = rospy.get_param("/ip_address")
    socket0 = classes.SocketLoomo(8080, dt_control/4, ip_address, packer="f f")
    #socket6 = SocketLoomo(8086, dt_control/4, ip_address, unpacker="f f")

    # Initialize control parameters
    CONTROL_FUNCTION = rospy.get_param("/CONTROL_FUNCTION")
    wheel_base = rospy.get_param("/wheel_base") # m
    v_max = rospy.get_param("/v_max") # m/s
    loomo = classes.MobileRobot(wheel_base, v_max)
    control_prediction_horizon = rospy.get_param("/time_horizon_control") # s

    if CONTROL_FUNCTION == "Default":
        controller = MPC_Control.MPC(loomo, dt_control, control_prediction_horizon)
        N = controller.N

    elif CONTROL_FUNCTION == "EPFL_Driverless":
        controller = MPC_Eloi.MPC_model()
        N = controller.HORIZON_N

    n_states = rospy.get_param("/n_states")

    # Initialize control variables
    global local_path, state, x0, planner_counter, local_predictions, global_path

    actual_state = [0.0] * n_states
    actual_path_local = []
    state = [0.0] * n_states
    state_planner = state
    local_path = []
    global_path = []
    planner_counter = 0
    predicted_states_local = []
    local_predictions = []

    rospy.loginfo("Control Node Ready")
    rospy.sleep(3.)

    while not rospy.is_shutdown():
        start = time.time()
        control_cmd = [0.0, 0.0]

        if len(local_path) > int(N) and len(global_path) > int(N):
            # Initial planner state and Actual state in global coordinates
            state_planner = x0
            actual_state = state
            # Actual state in relation to first path planning position
            state_local = transformations.Global_to_Local(state_planner, [actual_state])[0]

            # Desired states we need for the control algorithm
            planner_counter_l = utilities.minimum_distance(state_local, local_path)
            planner_counter_g = utilities.minimum_distance(actual_state, global_path)
            actual_path_local = local_path[planner_counter_l:]
            actual_path_global = global_path[planner_counter_g:]
            desired_path_local = actual_path_local[:N+1]
            desired_path_global = actual_path_global[:N+1]
            local_predictions = local_predictions

            # Calculate the actual control command and the control path it predicts to follow.
            if len(desired_path_global)>int(N):

                if CONTROL_FUNCTION == "Default":
                    control_cmd, predicted_states_local = MPC_Control.mpc_control_loomo(controller, state_local, desired_path_local)

                elif CONTROL_FUNCTION == "EPFL_Driverless":
                    controller.acquire_path(desired_path_global)
                    control_cmd, predicted_states_global = controller.run_MPC(actual_state)
                    predicted_states_local = transformations.Global_to_Local(actual_state, predicted_states_global)

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
        print(values)
        
        # Calculate node computation time
        computation_time = time.time() - start
        
        if computation_time > dt_control:
            rospy.logwarn("Control computation time higher than node period by " + str(computation_time-dt_control) + " seconds")

        rate.sleep()

    values = (0.0, 0.0)
    socket0.sender(values)

if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        dt_control = rospy.get_param("/dt_control")
        ip_address = rospy.get_param("/ip_address")
        socket0 = classes.SocketLoomo(8080, dt_control/4, ip_address, packer="f f")
        values = (0.0, 0.0)
        socket0.sender(values)
        pass