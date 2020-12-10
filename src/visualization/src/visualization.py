#!/usr/bin/env python
# VITA, EPFL
import rospy
from msg_types.msg import Position, PositionArray, TrajectoryArray, StateArray, State
import matplotlib.pyplot as plt
import numpy as np

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath('/home/cconejob/StudioProjects/socket-loomo/src/perception/scripts/tools')))
from tools.classconverter import *
from tools.classes import *
from tools.transformations import *
from tools.utilities import *


def callback_planning_local(data):

    global planning_local, planning_global, start_node, x0

    x = x0

    if start_node:
        planning_local = StateArray2list(data)
        planning_global = Local_to_Global(x, planning_local)


def callback_planning_state(data):

    global x0

    x0 = State2list(data)


def callback_control_local(data):

    global control_local, control_global, start_node, state_global, x0

    x = x0

    if start_node:
        control_local = StateArray2list(data)
        control_global = Local_to_Global(x, control_local)


def callback_estimation(data):

    global state_local, state_global, goal_local, goal_global, start_node

    state_global = State2list(data)
    state_local = [0.0, 0.0, 0.0]
    goal_local = Global_to_Local(state_global, [goal_global], True)[0]
    start_node = True


def callback_prediction(data):

    global predictions_local, predictions_global, state_global

    x = x0
    predictions_local = TrajectoryArray2list(data)
    predictions_global = Local_to_Global_prediction(x, predictions_local)


def visualization_function(predictions, planning, control, state, goal):
    plt.clf()
    plt.plot(goal[0], goal[1], "rx")
    plt.plot(state[0], state[1], "ro")
    plt.arrow(state[0], state[1], 0.1*np.cos(state[2]), 0.1*np.sin(state[2]), width=0.02)

    if len(predictions) > 0:

        for e in predictions:

            for i in range(len(e)):

                if len(e)==1:
                    plot_circle(e[i][0],e[i][1], 0.25,"g-")

                elif i == 0 and e!=[[]] and e[i][0]!=0.0:
                    plot_circle(e[i][0],e[i][1], 0.25,"b-")

                else:
                    plot_circle(e[i][0],e[i][1], 0.25,"b--")

    debug_activated = False

    if debug_activated:
        rospy.logwarn("-----------------NEW VISUALIZATION-------------------")
        rospy.loginfo("PLANNER: " + str(planning[:len(control)]))
        rospy.loginfo("PREDICTED STATES: " + str(control))
        rospy.loginfo("STATE: " +str(state))
        rospy.loginfo("OBJECTS: " + str(predictions))

    plt.axis([-2, 4, -3, 3])
    plt.grid(True)
    plt.pause(0.001)
    plt.plot([x for (x, y, heading) in planning], [y for (x, y, heading) in planning], 'r--')
    plt.pause(0.001)
    plt.plot([x for (x, y, heading) in control], [y for (x, y, heading) in control], 'g-.')
    plt.pause(0.001)


def main():
    # Initialize ROS visualization node
    rospy.init_node("visualization")
    visualization_activated = rospy.get_param("/perception/visualization_activated")
    if visualization_activated:
        sub_planning = rospy.Subscriber('/Visualization/desired_states_local', StateArray, callback_planning_local, queue_size = 1)
        sub_control = rospy.Subscriber('/Visualization/predicted_states_local', StateArray, callback_control_local, queue_size = 1)
        sub_estimation = rospy.Subscriber('/Visualization/estimated_state', State, callback_estimation, queue_size = 1)
        ub_estimation = rospy.Subscriber('/Visualization/planner_state', State, callback_planning_state, queue_size = 1)
        sub_prediction = rospy.Subscriber('/Visualization/predictions_local', TrajectoryArray, callback_prediction, queue_size = 1)

    dt_visualization = rospy.get_param("/control/dt_control")
    rate = rospy.Rate(int(1/dt_visualization))

    global planning_local, predictions_local, control_local, state_local, state_global, goal_local, goal_global, start_node, x0

    state_local = []
    state_global = []
    predictions_local = []
    planning_local = []
    control_local = []
    goal_global = [3.0, 1.0]
    state_local = [0.0, 0.0, 0.0]
    x0 = [0.0, 0.0 , 0.0]

    visualization = True
    local = False

    start_node = False

    rospy.loginfo("Visualization Node Ready")
    rospy.sleep(4.)

    while not rospy.is_shutdown() and visualization_activated:

        if visualization and start_node and local:
            visualization_function(predictions_local, planning_local, control_local, state_local, goal_local)

        if visualization and start_node and not local:
            visualization_function(predictions_global, planning_global, control_global, state_global, goal_global)

        rate.sleep()


if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass