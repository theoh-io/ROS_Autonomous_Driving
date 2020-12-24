#!/usr/bin/env python
# VITA, EPFL
import rospy

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath('/home/cconejob/StudioProjects/socket-loomo/src/perception/scripts/tools')))
from tools import transformations, utilities


class SlamConfiguration:

    def __init__(self, range_sensor, error_sensor):
        self.n_landmarks = 0
        self.range_sensor = range_sensor
        self.error_sensor = error_sensor
        self.saved_map = []
        self.map_state = []


    def mapping(self, state, position_objects_local):
        position_objects_global = transformations.Local_to_Global(state, position_objects_local, True)

        for i in range(len(position_objects_local)):
            distance = utilities.calculate_distance(position_objects_local[i])

            if distance <= self.range_sensor:
                self.check_if_position_in_map(position_objects_global[i], 0, True)

        return self.saved_map, self.map_state


    def check_if_position_in_map(self, object_global, i=0, first_check=True):
        idx = i

        for object_map in self.saved_map[idx:]:
            if idx >= len(self.saved_map):
                break

            if len(object_map) > 0:
                distance = utilities.calculate_distance(object_global, object_map)

            else:
                return

            if distance <= self.error_sensor:
                #rospy.loginfo("data associated")
                #rospy.loginfo(object_global)

                if first_check:
                    self.saved_map[idx] = [object_global[0], object_global[1], idx+1]
                    #check_if_position_in_map(self, object_global, idx+1, first_check=False)

                else:
                    rospy.loginfo("object " + str(idx) + " being removed")
                    _ = self.saved_map.pop(idx)

                return

            idx = idx + 1

        if first_check:
            self.n_landmarks = len(self.saved_map) + 1
            self.saved_map.append([object_global[0], object_global[1], self.n_landmarks])


