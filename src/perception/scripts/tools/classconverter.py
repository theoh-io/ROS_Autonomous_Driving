#!/usr/bin/env python3
# VITA, EPFL
from msg_types.msg import PositionArray, Position, TrajectoryArray, State, StateArray, ControlCmd
from copy import deepcopy


def list2State(data):
    state = State()
    state.x = data[0]
    state.y = data[1]
    state.heading = data[2]

    return state


def list2StateArray(data):
    array_path = StateArray()
    state_array = PositionArray()
    array_path.desired_path = []
    state_array.objects = []

    for j in data:
        state = State()
        state.x = j[0]
        state.y = j[1]
        state.heading = j[2]
        array_path.desired_path.append(state)

    return array_path


def list2Position(data):
    position = Position()
    position.x = data[0]
    position.y = data[1]

    return position


def list2PositionArray(data):
    perception_command = PositionArray()
    perception_command.objects = []

    for i,object in enumerate(data):
        object_i = Position()
        object_i.x = object[0]
        object_i.y = object[1]
        object_i.t = 0.0
        object_i.id = object[2]
        object_i.actual = True
        perception_command.objects.append(object_i)

    return perception_command


def list2TrajectoryArray(data):
    predicted_trajectories_cmd = TrajectoryArray()
    print(list)
    for idx, detection in enumerate(data):
        prediction_object_i = PositionArray()

        for j in range(len(detection)):
            prediction_j = Position()
            prediction_j.x = detection[j][0]
            prediction_j.y = detection[j][1]
            prediction_j.t = detection[j][2]
            prediction_j.id = idx + 1
            prediction_j.actual = True

            prediction_object_i.objects.append(prediction_j)

        predicted_trajectories_cmd.trajectories.append(prediction_object_i)

    return predicted_trajectories_cmd


def list2ControlCmd(data):
    cmd = ControlCmd()
    cmd.v = data[0]
    cmd.w = data[1]

    return cmd


def Position2list(data):

    return [data.x, data.y]
    

def State2list(data):
    
    return [data.x, data.y, data.heading]


def StateArray2list(data):
    planning = []

    for e in data.desired_path:
        planning.append([e.x, e.y, e.heading])

    return planning


def PositionArray2list(data):
    pos_objects = []

    for i in data.objects:
        if i.x != 0.0 and i.y != 0.0:
            pos_objects.append([i.x, i.y, i.id, i.actual])

    if len(pos_objects) == 0:
        pos_objects = [[]]

    return pos_objects


def TrajectoryArray2list(data):
    array_predictions = []

    for e1 in data.trajectories:
        one_object_prediction = []

        for e2 in e1.objects:
            one_object_prediction.append([e2.x, e2.y, e2.t, e2.id])

        array_predictions.append(one_object_prediction)

    return array_predictions


def PositionArray2TrajectoryArraylist(data):

    traj_objects = []

    for i in data.objects:
        if i.x != 0.0 and i.y != 0.0:
            traj_objects.append([[i.x, i.y, i.id, i.actual]])

    if len(traj_objects) == 0:
        traj_objects = []

    return traj_objects