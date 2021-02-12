#!/usr/bin/env python3
# VITA, EPFL
import numpy as np
from copy import deepcopy

'''
Transformation from local to global coordinates:
State: actual position in global coordinates. 
Array_local: vector in local coordinates.
'''
def Local_to_Global (state, array_local, is_object = False):
    # Define rotation matrix
    R_matrix = np.array([[np.cos(state[2]), -np.sin(state[2])], [np.sin(state[2]), np.cos(state[2])]])

    # Define translation vector
    T_vec = np.array([state[0], state[1]])

    # Extract every (x,y) of the list
    array_local_pos = [[pos[0], pos[1]] for pos in array_local]

    # X_global = R * X_local + T
    array_global_pos = np.array([ np.matmul(R_matrix, pos) + T_vec for pos in array_local_pos])

    # Initialize final path
    global_array_path = deepcopy(array_local)

    for it in range(len(array_local)):
        # Update final path
        global_array_path[it][0] = array_global_pos[it, 0]
        global_array_path[it][1] = array_global_pos[it, 1]

        # If angles need to be rotated
        if not is_object:
            global_array_path[it][2] += state[2]

            # Angles always between [-pi, pi]
            if global_array_path[it][2] > 3.1415:
                global_array_path[it][2] -= 2*3.1415

            if global_array_path[it][2] < -3.1415:
                global_array_path[it][2] += 2*3.1415

    return global_array_path


'''
Transformation from local to global coordinates:
State: actual position in global coordinates. 
Array_local: predictions in local coordinates.
'''
def Local_to_Global_prediction(state, array_local):
    # Define rotation matrix
    R_matrix = np.array([[np.cos(state[2]), -np.sin(state[2])], [np.sin(state[2]), np.cos(state[2])]])

    # Define translation vector
    T_vec = np.array([state[0], state[1]])

    # X_global = R * X_local + T
    global_array_path = []

    for array in array_local:
        global_array = []

        for e in array:
            # Update final path
            vec = np.array([e[0], e[1]])
            vec = np.matmul(R_matrix, vec)
            vec += T_vec
            global_array.append(vec)

        global_array_path.append(global_array)

    if len(global_array_path)==0:
        global_array_path = [[]]

    return global_array_path


'''
Transformation from global to local coordinates:
State: actual position in global coordinates. 
Array_global: vector in global coordinates.
'''
def Global_to_Local (state, array_global, is_object = False):
    # Define rotation matrix and invert it
    matrix = np.array([[np.cos(state[2]), -np.sin(state[2])], [np.sin(state[2]), np.cos(state[2])]])
    matrix = np.linalg.inv(matrix)
    
    # Initialize final path
    local_array_path = []

    for e in array_global:
        # X_local = inv(R) * (X_global - T)
        vec = np.array([e[0], e[1]])
        vec -= np.array([state[0],state[1]])
        vec = np.matmul(matrix, vec)
        vec = [vec[0], vec[1]]

        # If angles need to be rotated
        if not is_object:
            vec = [vec[0], vec[1], e[2]-state[2]]

            # Angles always between [-pi, pi]
            if vec[2] > 3.1415:
                vec[2] -= 2*3.1415
            if vec[2] < -3.1415:
                vec[2] += 2*3.1415

        local_array_path.append(vec)

    if len(local_array_path)==0:
        local_array_path = [[]]

    return local_array_path


'''
Transformation from global to local coordinates:
State: actual position in global coordinates. 
Array_global: predictions in global coordinates.
'''
def Global_to_Local_prediction(state, array_global):
    # Define rotation matrix and invert it
    matrix = np.array([[np.cos(state[2]), -np.sin(state[2])], [np.sin(state[2]), np.cos(state[2])]])
    matrix = np.linalg.inv(matrix)

    # Initialize final path
    local_array_path = []

    for array in array_global:
        local_array = []

        for e in array:
            # X_local = inv(R) * (X_global - T)
            vec = np.array([e[0], e[1]])
            vec -= np.array([state[0],state[1]])
            vec = np.matmul(matrix, vec)
            vec = [vec[0], vec[1], e[2], e[3]]

            local_array.append(vec)

        local_array_path.append(local_array)

    if len(local_array_path)==0:
        local_array_path = [[]]

    return local_array_path


'''
Rotation in the z-axis.
'''
def rotate(positions, theta):
    theta = np.deg2rad(theta)
    zr = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    return np.dot(positions, zr)