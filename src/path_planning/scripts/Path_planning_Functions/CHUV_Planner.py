#!/usr/bin/env python3
# VITA, EPFL

from scipy.interpolate import UnivariateSpline
import math
import matplotlib.pyplot as plt
import numpy as np
import rospy
import collections

import os
import sys
abs_path_to_tools = rospy.get_param("/abs_path_to_tools")
sys.path.append(os.path.dirname(os.path.abspath(abs_path_to_tools)))
from tools import classes, transformations, utilities

# Class for Patient-Following Path Planning
class CHUV_Planner:

    def __init__(self, mobile_robot, speed, N, dt_control, robot_position):
        # Autonomous System and control specifications
        self.mobile_robot = mobile_robot
        self.prediction_list = []
        self.speed = speed
        self.dt_control = dt_control
        self.N = N

        # Relative Position between Robot and Patient (In the straight scenario: y = 0.0)
        self.x_limit = 0.0
        self.y_limit = 1.0

        # Details for Curvilinear Path Planning
        self.robot_position = robot_position
        self.past_person_array_global = collections.deque(maxlen=5)
        self.past_heading = 0.0


    # Straight Patient-Following Path Planning
    def path_planning_straight(self, person_prediction_global, x0):
        t_max = 0.0
        person_array = np.array(person_prediction_global[0])
        
        # Goal global position
        x = person_array[0] - self.x_limit
        y = 0.0

        goal = [x, y]
        path = [[x0[0],x0[1]], goal]

        T_total = abs(x/self.speed)

        N = int(T_total/self.dt_control)

        # Draw a straight line
        if T_total > t_max:
            m = goal[1]/goal[0]
            x_list = np.linspace(x0[0], goal[0], num=N)
            y_list = m * x_list
            path = []

            for i in range(len(x_list)):
                path.append([x_list[i], y_list[i]])

        # Convert the straight line to MPC feasible path
        mpc_path_global = utilities.MPC_Planner_restrictions_CHUV_straight(self.mobile_robot, path, self.speed, self.dt_control, x0=x0)[:N+5]

        if len(mpc_path_global)<self.N:

            return [[0.0, 0.0, 0.0, 0.0], ] * 2 * self.N

        # Transform from Global to Local coordinates
        mpc_path = transformations.Global_to_Local(x0, mpc_path_global)

        return mpc_path


    # Curvilinear Patient-Following Path Planning
    def path_planning_curvilinear(self, person_prediction_global, x0, path_type="safe"):
        path = []
        t_max = 0.1
        person_array_global = np.array(person_prediction_global[0])
        heading = self.past_heading

        if len(self.past_person_array_global) > 1:

            if  utilities.new_heading_required(self.past_person_array_global[0], person_array_global):
                heading = math.atan2((person_array_global[1] - self.past_person_array_global[0][1]),(person_array_global[0] - self.past_person_array_global[0][0]))

            else:
                print("patient not moving")

        person_array_global[2] = heading
        self.past_person_array_global.append(person_array_global)
        self.past_heading = heading

        #print("object = " + str(person_array_global))

        if self.robot_position == "right":
            
            if abs(heading) <= math.pi/2:
                goal_global = transformations.Local_to_Global(person_array_global, [[self.x_limit, -self.y_limit, 0.0]])[0]

            else:
                goal_global = transformations.Local_to_Global(person_array_global, [[self.x_limit, -self.y_limit, 0.0]])[0]
        
        else:
            if abs(heading) <= math.pi/2:
                goal_global = transformations.Local_to_Global(person_array_global, [[self.x_limit, self.y_limit, 0.0]])[0]

            else:
                goal_global = transformations.Local_to_Global(person_array_global, [[self.x_limit, self.y_limit, 0.0]])[0]

        x = goal_global[0]
        y = goal_global[1]

        x_list = np.array([x0[0], x])
        y_list = np.array([x0[1], y])

        heading_line = math.atan2((y-x0[1]),(x-x0[0]))
        #print("heading line = " + str(heading_line))

        path = []

        if path_type == "linear":
            T_total = abs(utilities.calculate_distance(x0, goal_global)/self.speed)
            N = int(T_total/self.dt_control)
            m = (x0[1]-goal_global[1])/(x0[0]-goal_global[0])
            n = goal_global[1] - m * goal_global[0]
            x_list = np.linspace(x0[0], goal_global[0], num=N)
            y_list = m * x_list + n

        elif path_type == "safe":
            goal_global_2 = transformations.Local_to_Global(goal_global, [[-0.5,0.0,0.0]])[0]
            T_total1 = abs(utilities.calculate_distance(x0, goal_global_2)/self.speed)
            N1 = int(T_total1/self.dt_control)
            print("intermediate goal = " + str(goal_global_2))
            m1 = (x0[1] - goal_global_2[1])/(x0[0] - goal_global_2[0])
            n1 = goal_global_2[1] - m1 * goal_global_2[0]
            x_list1 = np.linspace(x0[0], goal_global_2[0], num=N1+2)
            y_list1 = m1 * x_list1 + n1

            T_total2 = abs(utilities.calculate_distance(goal_global_2, goal_global)/self.speed)
            N2 = int(T_total2/self.dt_control)
            m2 = (goal_global_2[1] - goal_global[1])/(goal_global_2[0] - goal_global[0])
            n2 = goal_global[1] - m2 * goal_global[0]
            x_list2 = np.linspace(goal_global_2[0], goal_global[0], num=N2+2)
            y_list2 = m2 * x_list2 + n2

            #print(x_list1)
            #print(x_list2)

            x_list = list(x_list1) + list(x_list2)
            y_list = list(y_list1) + list(y_list2)

        for i in range(len(x_list)):
            path.append([x_list[i], y_list[i]])

        mpc_path_global = utilities.MPC_Planner_restrictions_CHUV_curvilinear(self.mobile_robot, path, self.speed, self.dt_control, x0=x0)
        goal_local = transformations.Global_to_Local(x0, [goal_global])[0]
        print("goal = " + str(goal_global))
        #print("goal = " + str(goal_local))

        if len(mpc_path_global)<self.N:

            return [[0.0, 0.0, 0.0, 0.0], ] * 2 * self.N, goal_local

        mpc_path_local = transformations.Global_to_Local(x0, mpc_path_global)

        print("path global = " + str(mpc_path_global))
        #print("path local = " + str(mpc_path_local[:N-1]))

        return mpc_path_local, goal_local


if __name__ == "__main__":

    try:
        path = [[0.0,0.0,0.0],[0.00001,0.00002,0.00005],[0.01,0.01,0.02],[0.01,0.01,0.02],[0.01,0.01,0.02]]
        x0 = [10.0, -10.0, -1.0]
        loomo = classes.MobileRobot(0.5, 0.5)
        chuv = CHUV_Planner(loomo, 0.5, 5, 0.4, "right")
        object_pos = [-7, -2, 0.0]

        for i in range(1000):
            print("iteration " + str(i))
            x0 = transformations.Local_to_Global(x0, [path[1]])[0]
            object_pos = [object_pos[0]-0.10*np.random.rand(1,1)[0][0], object_pos[1]-0.10*np.random.rand(1,1)[0][0], 0.0]
            print("state" + str(x0))
            if not (path[4][0] == 0.0 and path[4][1] == 0.0 and path[4][2] == 0.0):
                path, goal_local = chuv.path_planning_curvilinear([object_pos], x0, "safe")

            else:
                print("Robot reached the goal")
                break

            print("path local" + str(path[:2]))

    except rospy.ROSInterruptException:
        pass