#!/usr/bin/env python
# VITA, EPFL
import rospy

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath('/home/cconejob/StudioProjects/socket-loomo/src/perception/scripts/tools')))
from tools.classconverter import *
from tools.classes import *
from tools.transformations import *
from tools.utilities import *


class SlamConfiguration:

    def __init__(self, range_sensor, error_sensor):
        self.n_landmarks = 0
        self.range_sensor = range_sensor
        self.error_sensor = error_sensor
        self.saved_map = []


def mapping(slam, state, position_objects_local):
    position_objects_global = Local_to_Global(state, position_objects_local, True)

    for i in range(len(position_objects_local)):
        distance = calculate_distance(position_objects_local[i])

        if distance <= slam.range_sensor:
            check_if_position_in_map(slam, position_objects_global[i], 0, True)


def check_if_position_in_map(slam, object_global, i=0, first_check=True):
    idx = i

    for object_map in slam.saved_map[idx:]:
        if idx >= len(slam.saved_map):
            break

        if len(object_map) > 0:
            distance = calculate_distance(object_global, object_map)

        else:
            return

        if distance <= slam.error_sensor:
            rospy.loginfo("data associated")
            rospy.loginfo(object_global)

            if first_check:
                slam.saved_map[idx] = [object_global[0], object_global[1], idx+1]
                #check_if_position_in_map(slam, object_global, idx+1, first_check=False)

            else:
                rospy.loginfo("object " + str(idx) + " being removed")
                _ = slam.saved_map.pop(idx)

            return

        idx = idx + 1

    if first_check:
        slam.n_landmarks = len(slam.saved_map) + 1
        slam.saved_map.append([object_global[0], object_global[1], slam.n_landmarks])


