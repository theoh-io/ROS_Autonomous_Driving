# Visualization



## Topics Represented

All topics are represented in a global coordinate system, which origin belongs to the initial state.

* **IMU State**
Data directly provided by the inertial sensor. Blue point. Trace is plotted with a discontinuous blue line.

* **Estimated State** 
Only if Extended Kalman Filters or SLAM state estimations are used. Red point. If EKF and SLAM are not enabled, the blue and red points will be in the same place. Trace is also plotted with a continous red line.

* **Observations and Predictions**
Represents the map with continous green circles, current observation with blue continous circles, and predictions with blue discontinous circles.

* **Goal** 
The desired destination for our robot. Shown with a red cross.

* **Desired Path** 
Path planned represented with a red discontinous line.

* **Control future States** 
States predicted by Model Predictive Control. Plotted with a green continous line. 