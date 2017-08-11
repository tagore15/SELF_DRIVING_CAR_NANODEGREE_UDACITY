[//]: # (Image References)
[image1]: ./shift.png "coordinate shifting image"

## The Model
Our state consists of following 6 variables for each waypoints as:-   
```
(x(t), y(t), psi(t), v(t), cte(t), epsi(t))
```
x(t) - x coordinate at time t     
y(t) - y coordinate at time t     
psi(t) - Angle with reference to x-axis at time t     
v(t)   - velocity at time t     
cte(t) - cross-talk error at time t     
epsi(t) - error in psi at time t     

Our control (actuator) variables are:-     
steer(t) - steering applied at time t   
a(t) - acceleration/throttle at time t   

our vehicle kinematic model for time duration dt is as per below equations:-    
```
x(t+1) = x(t) + v(t) * cos(psi(t)) * dt
y(t+1) = y(t) + v(t) * sin(psi(t)) * dt
psi(t+1) = (psi(t) - v(t) * steer(t)/Lf * dt)
v(t+1) = v(t) + a(t)*dt
```

Lf in above equation was obtained by measuring the radius formed by running the vehicle in the simulator around in a circle with a constant steering angle and velocity on a flat terrain. It was tuned until the the radius formed by the simulating the model presented in the classroom matched the previous radius. This is the length from front to CoG that has a similar radius. Its value for given environment in this project is taken as 2.67.    

While we fit our model, we minimize following cost function which results in less deviation from reference waypoints while meeting specified velocity and not providing arbitrary actuator values for steering and throttle          
```
cost = Sum(10000*cte(t)^2 + 5000*epsi(t)^2 + 5*(v(t)-v_required)^2 + 5*steer(t)^2 + 5*a(t)^2 + 200*steer(t)^2 + 10*(a(t)^2)) 
```

## Timestep Length and Elapsed Duration (N & dt)
I keep the value of number of points (N) in predictive model to 10. This ensures that we take adequate points without overdoing and avoiding high latency of solver.     
I started with high value of dt as 1. Car was falling off track with this value despite trying multiple weights in cost equation. On visual inspection of points, I noticed that they are far away which is resulting in large CTE. I keep reducing value of dt and finally settled on following parameters    
```
N  = 10
dt = 0.1
```

## Polynomial Fitting and MPC Preprocessing

I shift the coordinate systems with respect to car as per below equations with car moving direction as x axis and origin at car current position.

```
wayX = (shift_x*cos(psi) + shift_y*sin(psi));
wayY = (shift_y*cos(psi) - shift_x*sin(psi));
```
Here shift_x and shift_y corresponds to difference between point and car's global x and y coodinates.      

Below diagram discussed in Udacity forums illustrate calculation behind above equation nicely    
![coordinate translation image][image1]

We fit a 3rd order polynomial through above waypoints. We assume CTE to be y-coordinate while epsi is assumed to be deviation from slope of polynomial.

## Model Predictive Control with Latency
There is latency in our system due to time taken by MPC solver. During the time MPC resolves the model equation, car in simulator has moved forward a little. To take account of this latency, I apply following update to state before sending it to MPC.
```
double latency = 0.1;
double x = v*latency;
double y = 0;
psi =  -1*v*steer_value/2.67*latency;
v = v + throttle_value*latency;
cte = polyeval(coeffs, x);
epsi =  psi-atan(coeffs[1] + 2*x*coeffs[2] + 3*coeffs[3]*pow(x, 2));
```
