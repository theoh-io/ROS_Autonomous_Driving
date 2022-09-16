# Parameters

Inside [Loomo.launch](./launch/Loomo.launch) [xml], users can change the most important parameters of the pipeline depending on the requirements of the test, on the server where is being launched, ...

## Global

<p align="center">

| Parameter   |  Description                                 |
|:---------:  |:-------------------------:                   |
| ip_address_robot  | IP Address Autonomous System (client)  |
| ip_address_neuro    | IP Address 2nd Client: NeuroProstethics Device (keypoint information) |
| abs_path_to_tools    | Global path to pipeline tools |
| v_max       | Robot's maximum speed [m/s]                  |
| wheel_base  | Robot's length between wheels [m] (0.57 for Loomo)           |
| speed                   | Constant speed of the robot [m/s]           |

</p>

## Perception

<p align="center">

| Parameter           |  Description                              |
|:---------:          |:-------------------------:                |
| PERCEPTION_FUNCTION | Detection algorithm used: Stark           |
| dt_perception       | Perception sampling time [s] (will just wait to finish if its above)             |
| downscale           | Downscale for robot images (check follow.cfg) |
| detector_size       | DSize of detection model (small, medium, large, xlarge) |
| tracking_confidence  | treshold for tracking confidence 0.8 (id switches) -> 0.99 (not able to track) |
| keypoints_activated  | Boolean to activate 3D Pose Estimation |
| save_keypoints_vid   | Create a mp4 video to view 3D keypoints |
| keypoints_logging    | saving the coordinates of the 3D keypoints in a CSV file |
| visualization_percep | Boolean to visualize the image of loomo and bounding box |
| visualization_3D_activated  | Boolean to visualize the 3D Pose in Real Time |
| verbose_percep       | int value level of printing [0;4]: 0(minimal)->2(runtime)->4(full/debugging)  |



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
| verbose_map             | Boolean to indicate if we want to print messages   |

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