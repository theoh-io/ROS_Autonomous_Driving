#!/usr/bin/env python
# VITA, EPFL
import numpy as np

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath('/home/cconejob/StudioProjects/socket-loomo/src/perception/scripts/tools')))
from tools.classconverter import *
from tools.classes import *
from tools.transformations import *
from tools.utilities import *

def state_with_initial_calibration(initial_state, state_global):

    state_local = Global_to_Local(initial_state, [state_global])

    if len(state_global) > 5:
        state_local_total = [state_local[0][0], state_local[0][1], state_local[0][2], state_global[3], state_global[4], state_global[5]]

    return state_local_total
