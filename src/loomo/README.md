# Parameters

## Global

| Parameter   |  Description                                 |
|:---------:  |:-------------------------:                   |
| ip_address  | IP Address server-client                     |
| speed       | Constant speed of the robot [m/s]            |
| goal_x      | Goal's x coordenate (from initial point) [m] |
| goal_x      | Goal's y coordenate (from initial point) [m] |
| v_max       | Robot's maximum speed [m/s]                  |
| wheel_base  | Robot's length between wheels [m]            |

## Perception

| Parameter           |  Description                              |
|:---------:          |:-------------------------:                |
| PERCEPTION_FUNCTION | Detection algorithm used                  |
| dt_perception       | Perception sampling time [s]              |

## Robot State

| Parameter           |  Description                                |
|:---------:          |:-------------------------:                  |
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
| dt_path_planning        | Path Planning sampling time [s]            |

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