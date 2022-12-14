# Path Planning

## path_planning.py

<center>

![alt text](./Images/Software_path_planning.png)

</center>

### Path planning method

The algorithm aims to calculate the desired trajectory for the mobile robot from a start to a goal. We have designed in VITA a function based on a well-known method used for obstacle avoidance, RRT*, including the observations' motion over time (predictions). Other methods can be implemented inside the pipeline by changing parameters.

* **Prediction RRT Star (Default)**

<p align="center">
<img src="./Images/RRTStar.gif" alt="drawing" width="500"/>
</p>

As we mentioned before, RRT* is an algorithm for obstacle avoidance. Our main goal is not to crash with other objects or humans, depending on the detector used. We have taken into consideration all predictions in order to generate the path.

First of all, we define vehicle's constraints:

``` python
loomo = classes.MobileRobot(wheel_base, v_max)
```

Afterwards, we convert the global coordinates of the goal into local:

``` python
goal_local = transformations.Global_to_Local(x0, [goal_global], True)[0]
```

Where ```x0``` is the current state (received from the state estimator), and ```goal_global``` contains the goal position from the initial state.

Finally, we generate the desired path using all the parameters required.  

``` python
path, goal_local = RRT_star.planner_rrt_star(loomo, objects_now, speed, dt_control, goal_local, N, prediction_activated=prediction_activated)
```

If ```prediction_activated``` was false, we only calculate the next states depending on the current objects' position, ```objects_now```.

* **Straight Patient-Following Path Planning**

<p align="center">
<img src="./Images/Straight.png" alt="drawing" width="300"/>
</p>

* **Curvilinear Patient-Following Path Planning**

<p align="center">
<img src="./Images/Curvilinear.png" alt="drawing" width="300"/>
</p>
