#!/usr/bin/env python
# VITA, EPFL
import numpy as np


def linear_prediction(present_past_detections, dt_estimation, time_horizon_prediction):
    list_predictions = []
    t = np.linspace(0.0, time_horizon_prediction, 3)

    for i in range(len(present_past_detections)):
        list_detection = []
        past_number = len(present_past_detections[i])
        id = present_past_detections[i][past_number-1][2]

        dx_1 = present_past_detections[i][past_number-1][0]-present_past_detections[i][past_number-2][0]
        dy_1 = present_past_detections[i][past_number-1][1]-present_past_detections[i][past_number-2][1]

        v_x = dx_1/dt_estimation
        v_y = dy_1/dt_estimation

        a_x = 0.0
        a_y = 0.0

        x_predicted = present_past_detections[i][past_number-1][0] + v_x*t + 0.5*a_x*t**2
        y_predicted = present_past_detections[i][past_number-1][1] + v_y*t + 0.5*a_y*t**2

        for i2 in range(len(x_predicted)):
            list_detection.append([x_predicted[i2], y_predicted[i2], t[i2], id])

        list_predictions.append(list_detection)

    return list_predictions