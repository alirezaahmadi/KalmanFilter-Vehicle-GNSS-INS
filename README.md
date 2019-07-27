# KalmanFilter-Vehicle-GNSS-INS
project is about the determination of the trajectory of a moving platform by using a Kalman filter. For this purpose a kinematic multi sensor system (MSS) is used, which is equipped with three fiber-optic gyroscopes and three servo accelerometers. Additionally, the MSS contains an accurate RTK-GNSS. The system is adapted to a trolley, which can be pushed by a human being, however, it is also possible to adapt the system to a car or another moving vehicle. During the exercise real measurements are recorded at the Campus Poppelsdorf (University of Bonn) with the system. The goal is to determine the trajectory of the trolley via Kalman filtering in 2D by using a simplified motion model (constant accelerations, constant angular rates, motion only possible along the x-axis of the body frame) as well as the observations of the IMU (i. e. accelerations and angular rates) and the GNSS receiver (2D positions).

<div align="center">
	<img src="/images/trolley.png" alt="trolley" width="400" title="nlos"/>
</div>

## Task Description
Determine of the trajectory of a moving platform by using a
Kalman filter combining the prediction of the system model
and the measurements of GPS, the accelerometer and the
gyroscope

## Idea

Kalman filter is an application of Bayesian estimation
technique.
It obtains optimal estimates using the deterministic
and stochastic properties of the system model and
measurements to execute recursive state estimation.
A recursive algorithm: The current estimate is updated
by using the previous best estimates as inputs
System model provides a prediction of the current
state
Measurements are used to correct the predicted
state

<div align="center">
	<img src="/images/kf.png" alt="kf" width="400" title="nlos"/>
</div>

he predicted result and the measurements are combined by assigning weights to the prediction and the measurements
The new estimate is the weighted mean of the predicted state and the measurements.
Combination of these two independent estimates of a particular variable to get the mean value is the core of the Kalman filtering process.


### Requirements of Kalman Filter

• System model linear in the previous state and the
control parameters.

• Measurement model linear in current state In the case of extended Kalman Filter, the requirement on linearity on the system model and the measurement model may be relaxed. An additional requirement: Linearise the non-linear system model and/or the non-linear measurement model

<div align="center">
	<img src="/images/kfg.png" alt="kfg" width="400" title="nlos"/>
</div>

#### Input parameters which need to be predefined:

<div align="center">
	<img src="/images/1.png" alt="1" width="400" title="nlos"/>
</div>

#### Parameters which need to be predefined for the algorithm:

<div align="center">
	<img src="/images/2.png" alt="2" width="400" title="nlos"/>
</div>

### Two key steps:
1) Prediction stage
2) Correction stage

<div align="center">
	<img src="/images/3.png" alt="3" width="400" title="nlos"/>
</div>

## Prediction stage

<div align="center">
	<img src="/images/4.png" alt="4" width="800" title="nlos"/>
</div>

## Correction stage

<div align="center">
	<img src="/images/5.png" alt="5" width="800" title="nlos"/>
</div>

### Influence of the predefined stochastic settings
* Kalman gain assigns the weights to the prediction and the measurements.
* The criteria for assigning the weights is the level of precision.
* This is achieved by comparing the noise matrices R and Q in the two steps.
* Visualise R and Q as scalars.
* Big R but small Q implies the prediction is not precise but the measurements are.
* High weight on Q but small weight on R

### Kalman Filter (a real-time algorithm)
* Number of iterations: n
where n is the total number of time steps we set for the inputs
* Runtime~O(n)
* Total runtime does not grow faster with the input size
* The runtime of each of the later steps remains bounded
* Its Possible for the inputs and outputs to be processed simultaneously

# Extended Kalman filter

* Relax the condition on the linearity of the motion model and the
sensor model
* Requires the application of linearisation of the non-linear models for
the update of the covariance matrixes
* The rest is similar to Kalman filter

<div align="center">
	<img src="/images/6.png" alt="6" width="600" title="nlos"/>
</div>

## PipeLine

<div align="center">
	<img src="/images/ExtendedKF.png" alt="ExtendedKF" width="600" title="ExtendedKF"/>
</div>

# Simplified 2D motion model

<div align="center">
	<img src="/images/motion.png"  width="600"/>
</div>