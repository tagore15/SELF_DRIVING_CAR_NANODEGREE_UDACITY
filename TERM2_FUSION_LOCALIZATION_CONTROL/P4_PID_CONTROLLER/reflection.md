In this project, I implemented controller which provides steering angle for self-driving car in simulator environment provided by Udacity. I implemented PID controller which has following 3 components    

* P (Proportional)    
  This component introduces steering that is proportional to cross talk error i.e. error from required path.   

* I (Integeral)    
  This component removes inherent bias in system e.g. there might be unaligned wheel. To mitigate this effect, we introduce this term which is basically cumulative sum of error.    

* D (Differentiation)    
  This component takes into account the fact that if car is travelling toward required reference then it may overshot in future. so this component introduces control proportional to change in error.   

PID controller equation is  
```
steering = Kp*currentError + Ki*TotalError + Kd*lastChangeinError
```

For PID controller, we need values of Kp, Ki, Kd in above equation. I manually tuned these parameters by trial and error with different parameter values and observing the pattern as I tinkered with these coefficients.    
 
First I keep all 3 coefficients to zero. I found that with positive value of Kp, car was not able to make any headway and was going off the road at starting. To make car steer in direction of error, I made the value of Kp negative. I was able to get car running and make progress in track with little adjustment of Kp=-0.2.   
With chosen value of Kp, car was able to proceed in track but was overshooting with respect to required trajectory and was veering off the track in sharp turns. To mitigate the wobbling around reference line, I introduced low negative value of Kd paramter. I keep decreasing value of Kd till car was able to pass sharp turns with reduced wobbling. Finally I settled on valued of Kd = -5.0. It seems to have adverse affect on further decrease.         
With chosen values of Kp and Kd, car was able to cross the lap. Our car doesn't seem to have any bias so I didn't need to tune Ki and kept it at zero.    
 
Finally I settled on following values of hyperparameters and car was able to complete the track:-     
```
Kp = -0.2
Ki = 0
Kd = -5.0
```

There is still possibility to tune above paramters further so that we can run car at higher speeds.   

Check video file output.webm for a sample of car behavior with this PID controller at a sharp turn.   
