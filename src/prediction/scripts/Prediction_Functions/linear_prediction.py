#!/usr/bin/env python3
# VITA, EPFL
import numpy as np

class LinealPredictor():

    def __init__(self, dt=1.0, pred_horizon=1.0, obs_length=1):
        self.dt = dt
        self.pred_horizon = pred_horizon
        self.obs_length = obs_length

    def prediction_function(self, present_past_detections):
        list_predictions = []
        t = np.linspace(0.0, self.pred_horizon, int(self.pred_horizon/self.dt))

        for detection in present_past_detections:
            list_detection = []
            past_number = len(detection)

            dx_1 = detection[past_number-1][0]-detection[past_number-2][0]
            dy_1 = detection[past_number-1][1]-detection[past_number-2][1]

            v_x = dx_1/self.dt
            v_y = dy_1/self.dt

            x_predicted = detection[past_number-1][0] + v_x*t
            y_predicted = detection[past_number-1][1] + v_y*t

            for i2 in range(len(x_predicted)):
                list_detection.append([x_predicted[i2], y_predicted[i2], t[i2]])

            list_predictions.append(list_detection)

        return list_predictions