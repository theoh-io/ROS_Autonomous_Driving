# Prediction

## prediction.py

<center>

![alt text](./Images/Software_prediction.png)

</center>

### Predictor

It is an optional algorithm to predict the trajectory of the detections. It only runs if ```prediction_activated``` in loomo.launch is set to:

```html
<param name="prediction_activated" value="True" />
``` 

We offer two different predictors, both built in VITA laboratory: 

* **Linear (Default)** 

Linear fast trajectory predictor. It assumes that future states depend linearly on previous ones. Based on kinematics. Here we show the predictor's configuration:

```python
predictor = linear_prediction.LinealPredictor(dt=dt_prediction, pred_horizon=time_horizon_prediction, obs_length=n_past_observations)
``` 
  
* **trajNet++** 

Human trajectory predictor. Reference: https://github.com/vita-epfl/trajnetplusplusbaselines

In order to configure it, we require the following function:

```python
predictor = trajnetplus_predictor.TrajNetPredictor(dt=dt_prediction, pred_horizon=time_horizon_prediction, obs_length=n_past_observations, model_name=model_name, model="Occupancy", tag=2)
```

Where ```model_name``` can be set to ```"SGAN"``` or  ```"LSTM"``` architecture, and the other parameters can be changed if needed in loomo.launch.

Both detectors require previous ```n_past_observations```  of the object/person, and also past and current detections: 

```python
past_detections, past_present_positions = Utils.add_detections_to_past(detections, past_detections, n_past_observations)
predicted_trajectories = predictor.prediction_function(past_present_positions)
```

We recommend to activate both mapping and prediction in order to generate future trajectories because the generated map assotiates the current with the previous observations. This fact helps to identify every different person even if there is motion.

