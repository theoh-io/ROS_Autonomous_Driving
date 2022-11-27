#!/usr/bin/env python3
# VITA, EPFL
import rospy

import os
import sys
import rospkg
rospack=rospkg.RosPack()
abs_path_to_loomo=rospack.get_path('loomo')
abs_path_to_tools=abs_path_to_loomo+"/scripts/tools"
sys.path.append(os.path.dirname(os.path.abspath(abs_path_to_tools)))
from tools import transformations
from tools.utils import Utils, Plotting


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
            distance = Utils.calculate_distance(position_objects_local[i])

            # Check only if inside sensors range 
            if distance <= self.range_sensor:
                position_objects_global = transformations.Local_to_Global(state, position_objects_local, True)
                self.check_if_position_in_map(position_objects_global[i])

        return self.saved_map, self.map_state


    # Check if detection was observed before
    def check_if_position_in_map(self, object_global):
        print(f"object global {object_global}")
        print(f"saved map before {self.saved_map}")
        if not self.saved_map:
            self.n_landmarks = len(self.saved_map) + 1
            self.saved_map.append([object_global[0], object_global[1], self.n_landmarks])
            print(f"saved map after {self.saved_map}")
        for idx,object_map in enumerate(self.saved_map):
            #if len(object_map) > 0:
            distance = Utils.calculate_distance(object_global, object_map)
            print(f"distance association {distance}")
            # Data Assotiation if two detections in same range
            if distance <= self.error_sensor:
                self.saved_map[idx] = [object_global[0], object_global[1], idx+1]

            else:
                # Add new detection to map if it cannot be assotiated
                self.n_landmarks = len(self.saved_map) + 1
                self.saved_map.append([object_global[0], object_global[1], self.n_landmarks])
                print(f"saved map after {self.saved_map}")
    
