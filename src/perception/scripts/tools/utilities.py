#!/usr/bin/env python3
# VITA, EPFL
import numpy as np
import math
import matplotlib.pyplot as plt
import collections

'''
Plot cicrles --> Object mapping, detection and prediction plot.
'''
def plot_circle(x, y, size, color="b-"):
    deg = list(range(0, 360, 5))
    deg.append(0)
    xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
    yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
    plt.plot(xl, yl, color)


'''
Calculate distance between 2 objects
'''
def calculate_distance(object1, object2=[0.0, 0.0]):

    return math.sqrt((object1[0]-object2[0])**2 + (object1[1]-object2[1])**2)


'''
Calculate the index of the minim distance between array_path and state
'''
def minimum_distance(state, array_path):
    distances = []

    for e in array_path:
        dist_x = (e[0] - state[0])**2
        dist_y = (e[1] - state[1])**2
        dist_heading = (e[2] - state[2])**2
        dist_total = dist_x + dist_y + dist_heading
        distances.append(dist_total)

    val, idx = min((val, idx) for (idx, val) in enumerate(distances))

    return idx


'''
Add new detections to the past detections buffer
'''
def add_detections_to_past(pos_detections, past_pos_detections, past_number): # past_pos_detections = [deque([x1(t-past_n),y1(t-past_n)],...,[x1(t-1),y1(t-1)]),...,deque([xN(t-past_n),yN(t-past_n)],...,[xN(t-1),yN(t-1)])]

    list_idx = []
    past_for_predictions = []
    objects_detected = False

    if len(pos_detections)>0:
        objects_detected = True

    if objects_detected:
        if len(pos_detections[0])>0:

            for detection in pos_detections:
                list_idx.append(detection[2]-1)

                if detection[2] > len(past_pos_detections):
                    d = collections.deque(maxlen=past_number)

                    for i in range(past_number):
                        d.append([detection[0],detection[1],detection[2]])

                    past_pos_detections.append(d)

                else:
                    past_pos_detections[detection[2]-1].append([detection[0],detection[1], detection[2]])

    for idx in range(len(past_pos_detections)):

        if idx not in list_idx:
            past_pos_detections[idx].append([past_pos_detections[idx][past_number-1][0], past_pos_detections[idx][past_number-1][1], idx+1])
        
        else:
            past_for_predictions.append(list(past_pos_detections[idx]))

    return past_pos_detections, past_for_predictions