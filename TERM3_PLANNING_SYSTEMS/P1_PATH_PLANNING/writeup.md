[//]: # (Image References)

[image1]: ./images/waypoint.PNG
[image2]: ./images/frenet.png

The goal of this project is to create a virtual self driving car which navigates a highway. Car doesn't violate speed limit, max acceleration and jerk requirements for a comfort ride. It doesn't run slower than speed limit unless obstructed by traffic. It is also able to safely change lanes to pass slower running car.    

Speed limit of car for given track is 49.5 miles per hour. To move the car, we predict list of future points that car is expected to follow sequentially using its current velocity and yaw after every 0.02 seconds as shown in snapshot below.     
![waypoint][image1]   

We use Frenet coordinates to determine our location with reference to center of lane and other cars. We denote d coordinate to determine lane of car and s coordinate to determine position relative to other cars. Frenet coordinates make calculations easy and make sure that car keep its lane. Frenet coordinate system is depicted below:-     
![frenet][image2]   


To avoid sudden acceleration and jerk, we increase required velocity in units. We use spline library to smooth our path to avoid latitudinal jerks. We generate few endpoints for our next waypoints list and then plot a spline through these points. We can then find intermediate waypoints of our next trajectory through points lying on spline.      

We are using sensor fusion data to detect other cars on road. We find left, right and current lane information through d coordinates. If we deduct there is car ahead of us in current lane running at slower speed then we need to change lane to either left or right. We can change lane successfully if there is not a car running at slower speed in front or at higher speed from behind in new lane.    

This code determines if right lane is safe to change. As mentioned lane change is not possible, if there is a car running at high speed from behind or it is too close to change.

```
if ((check_car_s < car_s && check_speed > ref_vel) || car_s - check_car_s < 10)
{
    right_lane_close = true;
}
```

Similary for left lane, we can check if is safe to change lane through similar change.
```
if ((check_car_s < car_s && check_speed > ref_vel) || car_s - check_car_s < 10)
{
    left_lane_close = true;
}
```

If car is not able to switch lane, it reduces its speed. It changes speed to a safer lane whenever it finds its speed is lesser than velocity of forward car in right or left lane.     
```
if (current_lane_close)
{
    ref_vel -= .224;
    if (left_velocity > right_velocity)
    {
        if (lane != 0 && left_velocity > ref_vel && not left_lane_close)
        {
            lane--;
        }
        else if (lane != 2 && (right_velocity > ref_vel) && not right_lane_close)
        {
            lane++;
        }
    }
    else
    {
        if (lane != 2 && (right_velocity > ref_vel) && not right_lane_close)
        {
            lane++;
        }
        else if (lane != 0 && left_velocity > ref_vel && not left_lane_close)
        {
            lane--;
        }

    }
}
```

Our car was able to complete the track successfully with above software pipeline. There is scope of improvement by tuning threshold values and better prediction model. 
