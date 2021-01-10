# Visualization

## Topics Represented

All topics are represented in a global coordinate system, where the origin belongs to the initial state.

### IMU State

Data directly provided by the inertial sensor. Plotted with a blue point. Trace is plotted with a discontinuous blue line.

### Estimated State

Only if Extended Kalman Filters or SLAM state estimations are used and lotted with a red point. If EKF and SLAM are not enabled, the blue and red points will be in the same place. Trace is also plotted with a continuous red line.

### Observations and Predictions

Represents the map with continuous green circles, current observation with blue continuous circles, and predictions with blue discontinuous circles.

### Goal 

The desired destination for our robot. Shown with a red cross.

### Desired Path 

Path planned represented with a red discontinuous line.

### Control Future States

States predicted by Model Predictive Control. Plotted with a green continuous line.