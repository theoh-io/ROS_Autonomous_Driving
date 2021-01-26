#!/usr/bin/env python
# VITA, EPFL
import rospy

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath('/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/loomo/scripts/tools')))
from tools import transformations, utilities


class SlamConfiguration:

    def __init__(self, range_sensor, error_sensor):
        self.n_landmarks = 0
        self.range_sensor = range_sensor
        self.error_sensor = error_sensor
        self.saved_map = []
        self.map_state = []

    # Add detected object to map or check if its currently inside it
    def mapping(self, state, position_objects_local):

        for i in range(len(position_objects_local)):
            distance = utilities.calculate_distance(position_objects_local[i])

            # Check only if inside sensors range 
            if distance <= self.range_sensor:
                position_objects_global = transformations.Local_to_Global(state, position_objects_local, True)
                self.check_if_position_in_map(position_objects_global[i])

        return self.saved_map, self.map_state


    # Check if detection was observed before
    def check_if_position_in_map(self, object_global):

        for idx,object_map in enumerate(self.saved_map):

            if len(object_map) > 0:
                distance = utilities.calculate_distance(object_global, object_map)

                # Data Assotiation if two detections in same range
                if distance <= self.error_sensor:
                    self.saved_map[idx] = [object_global[0], object_global[1], idx+1]

                    return

        # Add new detection to map if it cannot be assotiated
        self.n_landmarks = len(self.saved_map) + 1
        self.saved_map.append([object_global[0], object_global[1], self.n_landmarks])


