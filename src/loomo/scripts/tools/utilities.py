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


# Apply Kinematic Restrictions to the planner, and adapt it for MPC.
def MPC_Planner_restrictions(mobile_robot, points, v, t):
    e = v*t
    plan = [[0.0, 0.0, 0.0, 0.0]]
    dist = 0.0
    it = 2
    x_ant = 0.0
    y_ant = 0.0
    heading_ant = 0.0

    for i in range(1,80):
        
        while dist < e and it < len(points):
            distAB = calculate_distance(points[it-2], points[it-1])
            dist = dist + distAB
            it = it + 1

        if dist >= e:
            it = it - 1
            d = dist - e
            prop = d/distAB
            x = points[it-1][0] - prop*(points[it-1][0] - points[it-2][0])
            y = points[it-1][1] - prop*(points[it-1][1] - points[it-2][1])
            heading = math.atan2((y - y_ant),(x - x_ant))

            if abs(heading-heading_ant) > mobile_robot.w_max * t:
                heading = heading_ant + np.sign(heading-heading_ant) * mobile_robot.w_max * t
                x = x_ant + e * np.cos(heading)
                y = y_ant + e * np.sin(heading)

            plan.append([x, y, heading, v])
            dist = d
            x_ant = x
            y_ant = y
            heading_ant = heading
            it = it + 1

    return plan


# Apply Kinematic Restrictions to the planner, and adapt it for MPC.
def MPC_Planner_restrictions_CHUV_straight(mobile_robot, points, v, t, x0=[0.0, 0.0, 0.0, 0.0]):
    e = v*t
    plan = [x0]
    dist = 0.0
    it = 2
    x_ant = 0.0
    y_ant = 0.0
    heading_ant = 0.0

    for i in range(1,80):
        
        while dist < e and it < len(points):
            distAB = calculate_distance(points[it-2], points[it-1])
            dist = dist + distAB
            it = it + 1

        if dist >= e:
            it = it - 1
            d = dist - e
            prop = d/distAB
            x = points[it-1][0] - prop*(points[it-1][0] - points[it-2][0])
            y = points[it-1][1] - prop*(points[it-1][1] - points[it-2][1])
            heading = math.atan2((y - y_ant),(x - x_ant))
            v = abs(v)

            if abs(heading-heading_ant) > mobile_robot.w_max * t:
                heading = 0.0
                x = x_ant - e * np.cos(heading)
                y = y_ant - e * np.sin(heading)
                v = -v

            plan.append([x, y, heading, v])
            dist = d
            x_ant = x
            y_ant = y
            heading_ant = heading
            it = it + 1

    return plan

def MPC_Planner_restrictions_CHUV_curvilinear(mobile_robot, points, speed, t, x0=[0.0, 0.0, 0.0, 0.0]):
    e = speed*t
    x0.append(0.0)
    plan = [x0]
    dist = 0.0
    it = 2
    x_ant = x0[0]
    y_ant = x0[1]
    heading_ant = x0[2]

    for i in range(1,80):
        
        while dist < e and it < len(points):
            distAB = calculate_distance(points[it-2], points[it-1])
            dist = dist + distAB
            it = it + 1

        if dist >= e:
            it = it - 1
            d = dist - e
            prop = d/distAB
            x = points[it-1][0] - prop*(points[it-1][0] - points[it-2][0])
            y = points[it-1][1] - prop*(points[it-1][1] - points[it-2][1])
            heading = math.atan2((y - y_ant),(x - x_ant))
            heading_act = heading
            v = speed
            
            #if abs(heading-x0[2]) <= math.pi/2:
            x = x_ant + e * np.cos(heading)
            y = y_ant + e * np.sin(heading)

            if abs(heading_act-heading_ant) > mobile_robot.w_max * t:
                heading = heading_ant + np.sign(heading-heading_ant) * mobile_robot.w_max * t
                v = 0.0
                x = x_ant
                y = y_ant

            #elif abs(heading-x0[2]) > math.pi/2:
                #v = -v
                #x = x_ant - e * np.cos(heading)
                #y = y_ant - e * np.sin(heading)
                #heading = heading - np.sign(heading) * math.pi

                #if abs(heading_act-heading_ant) > mobile_robot.w_max * t:
                    #heading = heading_ant + np.sign(heading-heading_ant) * mobile_robot.w_max * t

            plan.append([x, y, heading, v])
            dist = d
            x_ant = x
            y_ant = y
            heading_ant = heading
            it = it + 1

    return plan


def new_heading_required(past_person, person):
    return (calculate_distance(past_person, person)>0.3)


def GetClockAngle(v1, v2):
    # Product of 2 vector modules
    TheNorm = np.linalg.norm(v1)*np.linalg.norm(v2)
    # Cross product
    rho =  np.arcsin(np.cross(v1, v2)/TheNorm)
    # Dot multiplication
    theta = np.arccos(np.dot(v1,v2)/TheNorm)
    
    if rho < 0:
        return - theta
    else:
        return theta