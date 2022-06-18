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

# Example: Vehicle 2D motion model

<div align="center">
	<img src="/images/motion.png"  width="800"/>
</div>

## Prediction Stage:

<div align="center">
	<img src="/images/predict.png"  width="800"/>
</div>

<div align="center">
	<img src="/images/predict1.png"  width="800"/>
</div>

<div align="center">
	<img src="/images/predict2.png"  width="800"/>
</div>

<div align="center">
	<img src="/images/predict3.png"  width="800"/>
</div>

### Model does not match reality...

* Measurements have high precisions — Low variance
* BUT inaccurate — with large systematic bias
* Kalman filter assigns weight to different stages based on their relative levels of precision
* The output state will deviate significantly from the true state
* Because we put a lot of weight on the wrong model

### Measurement model

Types of Measurements:
* GPS measurements of the position
* Non-gravitational acceleration/specific force measurements with the accelerometer
* Angular rate measurements with the gyroscope Put these in a measurement vector z

<div align="center">
	<img src="/images/mes.png"  width="200"/>
</div>

### Correction Stage

Assume the true state is known,
the measurement and the true state are related by

<div align="center">
	<img src="/images/z.png"  width="150"/>
</div>

As the measurement model is linear,

<div align="center">
	<img src="/images/zk.png"  width="300"/>
</div>

<div align="center">
	<img src="/images/cor.png"  width="800"/>
</div>

<div align="center">
	<img src="/images/cor1.png"  width="800"/>
</div>

<div align="center">
	<img src="/images/cor2.png"  width="800"/>
</div>

<div align="center">
	<img src="/images/cor3.png"  width="800"/>
</div>

## Filter Tuning

Turning process adjusts a trade-off between trusting to predictions and measurements.

* This process has to be done through adjusting measurement and system noise models R and Q.
* In most of the cases R cold be set based on sensor specifications. ( it better to assume a bit more than values in datasheets )
* elements of Q have to set in an stochastic form ( they should be more than one ), depending on how much you think, the system model is precis or in could be affected by misalignment respect to real measurements and error sources.

## Results

<div align="center">
	<img src="/images/re.png"  width="800"/>
</div>

<div align="center">
	<img src="/images/res.png"  width="800"/>
</div>

## Comparison INS vs StrapDown

* The results from the strapdown integration starts to deviate from the measurements by GPS positioning very quickly.
* The strapdown integration was unable to track the trajectory accurately when there is a sharp turn.
* Despite some observable deviations, the computation by Kalman filter agrees with the GPS measurements very well.
* Manages to follow the trajectory without losing its accuracy in the event of a sharp turn.

<div align="center">
	<img src="/images/rescomp.png"  width="800"/>
</div>

## Why combine the two?

- Predictions based on the motion model can be output at
extremely small time intervals
-However, the system error accumulates and the updated
state loses its precision very quickly
- Measurements does not suffer from this growth of error. However, the measurements can only be made in much larger time intervals
 - By combining the prediction of the motion model and the
measurements, we can have data concerning the state at a very small time interval while limiting the growth in error

## Happy?


---

<div align="center">
  
[![paypal](https://pics.paypal.com/00/s/NGRhNWNlODUtMzZlOS00MjJhLTg2NDEtMzNiNzczMTZkMDU4/file.PNG)](https://www.paypal.com/donate/?hosted_button_id=23TQAZ9MSLAUU)

</div>

---
