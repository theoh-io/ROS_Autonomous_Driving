# Parameters

Inside [Loomo.launch](./launch/Loomo.launch) [xml], users can change the most important parameters of the pipeline depending on the requirements of the test, on the server where is being launched, ...

See the tradeoff section at the end.

## Global

<p align="center">

| Parameter   |  Description                                 |
|:---------:  |:-------------------------:                   |
| ip_address_robot  | IP Address Autonomous System (client)  |
| ip_address_nicolo    | Not used ! Second Client IP Address (keypoint information) |
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
| detector_size       | size of yolo model: small/medium/large/xlarge |
| tracking_confidence | Tracking conf score 0 (id switch)- 1 (loss of target) |
| keypoints_activated | true/false                                |
| save_keypoints_vid  | true/false                                |
| keypoints_logging   | saving keypoints coordinates in a CSV file   |
| visualization_percep| true/false camera loomo with bounding box   |
| visualization_3D_activated | true/false 3D keypoints visualization   |
| verbose_percep | 0 (minimal) ->4 (full) level of verbose, above 2 to see runtime  |



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
| verbose_map             | true/false                    |

</p>

## Prediction

<p align="center">

| Parameter               |  Description                                |
|:---------:              |:-------------------------:                  |
| PREDICTION_FUNCTION     | Prediction algorithm used                   |
| prediction_activated    | Prediction algorithm required?              |
| model_prediction_path   | global path to the prediction model         |
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

## Tradeoff :warning:

When tuning the parameters of the program one must be conscious about the coupling effects of some parameters. The dt_control is one of the most important one and should be approximately the same value as the runtime printed on the Loomo app so that motion planning and communications can be synchronized. It is intuitive that adding more components to the perception node will slow it's inference therefore it's important to adapt the speed of the robot to the dt_control value.

Some combinations of parameters tht seemed to work for me:
* Simple tracking: speed up to 0.7 m/s, keypoints not activated, dt_control around 0.25s
* 3D keypoints: speed around 0.3 m/s, keypoints and visualizations activated, dt_control around 0.4s
