# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Summary

In this project we design a model predictive control system to drive an autonomous car around a simulated track. 

## Car Model

The model of the car is described by the following equations. 

* x[t]    =  x[t-1] + v[t-1] * cos(psi[t-1]) * dt
* y[t]    =  y[t-1] + v[t-1] * sin(psi[t-1]) * dt
* psi[t]  =  psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
* v[t]    =  v[t-1] + a[t-1] * dt
* cte[t]  =  f(x[t-1]) - y[t-1] + v[t-1] * sin(epsilon[t-1]) * dt
* epsilon[t] =  psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt

The nomen clature is as follows:

* x, y are the vehicles's position
* psi is the vehicles's heading orientation
* v is the vehicles velocity
* cte is the cross-track error (distance of the vehicle's center line from the middle of the lane)
* epsilon is the orientation error


Furthermore, dt is the time step, and Lf corresponds to the distance between the mass center of the car and its front axle. Additionally,

* a is the throttle
* delta is the steering angle
* are the actuator inputs which are computed by the MPC controller.


## main.cpp

main.cpp starts out with the following values from the simulator

* ptsx is the x-position of waypoints ahead on the track in global coordinates
* ptsy is the y-position of waypoints ahead on the track in global coordinates
* px is the current x-position of the vehicle's position in global coordinates
* py is the current y-position of the vehicle's position in global coordinates
* psi is the current orientation angle of the vehicle
* v is the current velocity of the vehicle
* steering_angle is the current steering angle
* throttle is the current throttle

First all the values are transformed from the global coordinate system to the vehicle's coordinate system. The waypoint vectors ptsx and ptsy are transformed to the vehicle's coordinate system using the transformation matrix. The new waypoint vectors are called x_vehicle and y_vehicle. These two vectors specifcy the way the car is intended to follow.

Then, a third order polynomial is fitted to these waypoints using the polyfit function. Since the path is transformed to the car coordinate system, the position and orientation of the car, px, py and psi, are all zeros in the car coordinate system. The cross-track error, cte, is then calculated by evaluating the polynomial function at the current position of the car using polyeval. 

Latency
For latency, another step was added to predict the state of the system after 100 milliseconds of delay. This predicted state is then input to the controller. The state of the system after this 100 milliseconds delay can then be predicted.

Lastly, the state of the system and the polynomial coefficients are input to the MPC controller. The optimized control outputs (steering angle and throttle values) are computed by the controller mpc.solve function and are sent back to the simulator to keep the car on the desired path


## MPC.cpp
The objective of the controller is to minimize a cost function that depends on different factors including:

* Sum of square values of cte and epsi to minimize cross-track and orientation errors.
* Sum of square values of (v - v_ref) to minimize the difference of the speed with the reference speed.
* Sum of square of actuator values a and delta to penalize large actuator actions.
* Sum of square values of the difference between two consecutive actuator values to penalize sharp changes.

I used weight parameters to prioritize the importance of each factor in the cost function. The appropriate weight values were obtained by the try-and-error method. It was noticed that the weights corresponding to the steering angle input and its rate have the most significant impact on the performance of the system and choosing large values for these two weights (W_DELTA and W_DDELTA) helps improving the stability of the car and avoiding the erratic and sudden steering behavior. Using the following weight values, a smooth and safe behavior can be obtained.

#define W_CTE 2
#define W_EPSI 1
#define W_V 1
#define W_DELTA 3000
#define W_A 1
#define W_DDELTA 2000
#define W_DA 1

## Timestep Length and Elapsed Duration (N & dt):

The parameters N(the number of points) and dt define the prediction horizon. Choosing a long prediction horizon can theoretically improve the prediction; but in practice, it increases the computational complexity. With too many points, the controller becomes slower and can become unstable. After some try and error and experimenting different values, I found that the car behaves well with N = 10 and dt = 0.1 which corresponds to a 1-second time horizon.



