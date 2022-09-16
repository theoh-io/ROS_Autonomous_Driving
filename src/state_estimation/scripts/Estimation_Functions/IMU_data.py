#!/usr/bin/env python3
# VITA, EPFL
import rospy
import numpy as np

import os
import sys
import rospkg
rospack=rospkg.RosPack()
abs_path_to_loomo=rospack.get_path('loomo')
abs_path_to_tools=abs_path_to_loomo+"/scripts/tools"
sys.path.append(os.path.dirname(os.path.abspath(abs_path_to_tools)))
from tools.classconverter import *
from tools.classes import *
from tools.transformations import *

def state_with_initial_calibration(initial_state, state_global):

    state_local = Global_to_Local(initial_state, [state_global])

    if len(state_global) > 5:
        state_local_total = [state_local[0][0], state_local[0][1], state_local[0][2], state_global[3], state_global[4], state_global[5]]

    return state_local_total
