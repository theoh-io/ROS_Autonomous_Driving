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

    return state_local[0]
