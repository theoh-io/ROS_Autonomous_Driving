# Parameters

In this file, the user can change the most important parameters of the pipeline depending on the requirements of the test, on the server where is being launched, ...

## Global

| Parameter   |  Description                                 |
|:---------:  |:-------------------------:                   |
| ip_address_robot  | IP Address Autonomous System (client)  |
| v_max       | Robot's maximum speed [m/s]                  |
| wheel_base  | Robot's length between wheels [m]            |

## Perception

| Parameter           |  Description                              |
|:---------:          |:-------------------------:                |
| PERCEPTION_FUNCTION | Detection algorithm used                  |
| dt_perception       | Perception sampling time [s]              |
| ip_address_nicolo    | Second Client IP Address (keypoint information) |


## Robot State

| Parameter            |  Description                                |
|:---------:           |:-------------------------:                  |
| ROBOT_STATE_FUNCTION | Robot State algorithm used                 |
| dt_robot_state       | Robot State sampling time [s]              |

## Map State

| Parameter               |  Description                  |
|:---------:              |:-------------------------:    |
| MAP_STATE_FUNCTION      | Map State algorithm used      |
| mapping_activated       | Mapping algorithm required?   |
| map_state_activated     | Estimate the state with map?  |
| dt_map_state            | Map State sampling time [s]   |

## Prediction

| Parameter               |  Description                                |
|:---------:              |:-------------------------:                  |
| PREDICTION_FUNCTION     | Prediction algorithm used                   |
| prediction_activated    | Prediction algorithm required?              |
| time_horizon_prediction | Last predicted time [s]                     |
| past_observations       | Number of past observations needed          |
| dt_prediction           | Prediction sampling time [s]                |

## Path Planning

| Parameter               |  Description                                |
|:---------:              |:-------------------------:                  |
| PATH_PLANNING_FUNCTION  | Path Planning algorithm used                |
| dt_path_planning        | Path Planning sampling time [s]             |
| time_horizon_path_planning  | Last planned time for path calculation [s] |
| speed                   | Constant speed of the robot [m/s]           |
| planner_type            | Straight/Curvilinear path planning          |
| robot_position          | Right/Left relative to the patient          |

## Control

| Parameter               |  Description                  |
|:---------:              |:-------------------------:    |
| CONTROL_FUNCTION        | Control algorithm used        |
| time_horizon_control    | Last control MPC time [s]     |
| n_states                | Number of states for control  |
| dt_control              | Control sampling time [s]     |

## Visualization

| Parameter              |  Description                                 |
|:---------:             |:-------------------------:                   |
| VISUALIZATION_FUNCTION | Visualization algorithm used                 |
| dt_visualization       | Visualization sampling time [s]              |