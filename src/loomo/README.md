# Parameters

Inside [Loomo.launch](./launch/Loomo.launch) [xml], users can change the most important parameters of the pipeline depending on the requirements of the test, on the server where is being launched, ...

## Global

<p align="center">

| Parameter   |  Description                                 |
|:---------:  |:-------------------------:                   |
| ip_address_robot  | IP Address Autonomous System (client)  |
| ip_address_nicolo    | Second Client IP Address (keypoint information) |
| abs_path_to_tools    | Global path to pipeline tools |
| v_max       | Robot's maximum speed [m/s]                  |
| wheel_base  | Robot's length between wheels [m]            |
| speed                   | Constant speed of the robot [m/s]           |

</p>

## Perception

<p align="center">

| Parameter           |  Description                              |
|:---------:          |:-------------------------:                |
| PERCEPTION_FUNCTION | Detection algorithm used                  |
| dt_perception       | Perception sampling time [s]              |
| downscale           | Downscale for robot images (check follow.cfg) |


</p>

## Robot State

<p align="center">

| Parameter            |  Description                                |
|:---------:           |:-------------------------:                  |
| ROBOT_STATE_FUNCTION | Robot State algorithm used                 |
| dt_robot_state       | Robot State sampling time [s]              |

</p>

## Map State

<p align="center">

| Parameter               |  Description                  |
|:---------:              |:-------------------------:    |
| MAP_STATE_FUNCTION      | Map State algorithm used      |
| mapping_activated       | Mapping algorithm required?   |
| map_state_activated     | Estimate the state with map?  |
| dt_map_state            | Map State sampling time [s]   |

</p>

## Prediction

<p align="center">

| Parameter               |  Description                                |
|:---------:              |:-------------------------:                  |
| PREDICTION_FUNCTION     | Prediction algorithm used                   |
| prediction_activated    | Prediction algorithm required?              |
| time_horizon_prediction | Last predicted time [s]                     |
| past_observations       | Number of past observations needed          |
| dt_prediction           | Prediction sampling time [s]                |

</p>

## Path Planning

<p align="center">

| Parameter               |  Description                                |
|:---------:              |:-------------------------:                  |
| PATH_PLANNING_FUNCTION  | Path Planning algorithm used                |
| dt_path_planning        | Path Planning sampling time [s]             |
| time_horizon_path_planning  | Last planned time for path calculation [s] |
| planner_type            | Straight/Curvilinear path planning          |
| robot_position          | Right/Left relative to the patient          |
| goal                    | Goal (detection collision avoidance)        |
| workarea                | Boundaries (detection collision avoidance)  |

</p>

## Control

<p align="center">

| Parameter               |  Description                  |
|:---------:              |:-------------------------:    |
| CONTROL_FUNCTION        | Control algorithm used        |
| time_horizon_control    | Last control MPC time [s]     |
| n_states                | Number of states for control  |
| dt_control              | Control sampling time [s]     |

</p>

## Visualization

<p align="center">

| Parameter              |  Description                                 |
|:---------:             |:-------------------------:                   |
| VISUALIZATION_FUNCTION | Visualization algorithm used                 |
| dt_visualization       | Visualization sampling time [s]              |

</p>