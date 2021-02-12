#!/usr/bin/env python
# VITA, EPFL

from scipy.interpolate import UnivariateSpline
import math
import matplotlib.pyplot as plt
import numpy as np
import rospy

import os
import sys
abs_path_to_tools = "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/loomo/scripts/tools"
sys.path.append(os.path.dirname(os.path.abspath(abs_path_to_tools)))
from tools import classconverter, classes, transformations, utilities

# Class for RRT Star planning
class CHUV_Planner:

    def __init__(self, mobile_robot, speed, dt_control):
        
        self.mobile_robot = mobile_robot
        self.prediction_list = []
        self.speed = speed
        self.dt_control = dt_control

    def path_planning(self, prediction_list):
        path = []

        for detection in prediction_list:
            detection_array = np.array(detection)
            x = detection_array[:,0] - 1.0
            y = detection_array[:,1] - 1.0

            p = [[0.0, 0.0]] + [[a,b] for a, b in zip(x,y)]

            path.append(p)

        if len(path)<1:
            path.append([[0.0,0.0]*20])

        mpc_path = utilities.MPC_Planner_restrictions(self.mobile_robot, path[0], self.speed, self.dt_control)

        return mpc_path



if __name__ == "__main__":

    try:
        loomo = MobileRobot(0.5, 0.5)
        chuv = CHUV_Planner(loomo, 0.25, 0.6)
        path = chuv.path_planning([[[0,2],[1,2],[2,2],[3,2],[4,2]],[[3,2],[4,2],[5,2]]])
        print(path)

    except rospy.ROSInterruptException:
        pass