#**Term 2 : PID Controller Project**


**PID Controller Project**

The goals / steps of this project are the following:

[//]: # (Image References)

[image0]: ./../camera_cal/calibration1.jpg "calibration1.jpg"


## Rubric Points
###Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/824/view) individually and describe how I addressed each point in my implementation.  


###Explanation of P,I,D values

* Proportional part P = difference between the current value and the sensor value. This is used to compensate 
* Integral part I = sum of all subsequent errors in order to smoothen the movement
* Derivative part D = prediction of the last deviation into the future

Discussion on proportional part P
* P has a fast reaction time
* P tends to strong overshooting (e.g. only using P means strong oscillations which I observed when setting I=D=0)
* and thus tends to reduce the stability

Discussion on integral part I
* I decreases the steady-state error
* I tends to overshooting at higher speed
* and reduces stability at higher speed (see also discussion in next chapter)

Discussion on derivative part D
* it has no effect on the steady-state error
* it decreases overshooting
* and thus but increases stability 


###Discuss how you chose the final hyperparameters (P, I, D coefficients). This could be have been done through manual tuning, twiddle, SGD, or something else, or a combination!

First I tried to set the parameters manually as good as it was possible to drive around the track slowly. This worked out pretty well for the values Kp = 1 , Ki = 1, Kd = 1. This was purely try&error up to this point.

After being able to drive I've implemented a twiddle algorithm in two flavours:
* one as part of the main function main.cpp including resetting after 2500 iterations (which is nearly one track round)
* one as part of the PID controller PID.cpp using a separate worker thread

The first twiddle algorithm is implemented in the main method and resetting the simultation after 2500 iterations. This has the following advantages:
* the best error is always taken from the same part of the track
* in case the the car is off the track it will resetted and the twidlle algorithm continues from starting point again

The second flavor I've implemented because the first twiddle algorithm was slow and not very nice to look at. After having better and more stable hyperparameters the fine tuning was much better using a kind of "online twiddle" which is constantly improving the results.
Advantages of the second twiddle algorithm are:
* much faster
* more practical in terms of changing parameters and tweaking

However the second twiddle algorithm has major disadvantages:
* if the car is off the track it might not come back again (there is no reset!)
* the sum of CTE might not be calculated always on the same part of the track and thus might lead to wrong twiddle results (of course bad twiddle parameters will be corrected again after some time, but it might converge slower)

Finally I found out that also the integral part has a high potential for overshooting as the oscillations with the final parameters were still pretty strong. Thus I introduced the reset of the integral error in PID.cpp lines 44-48 when the integral error reaches a value close to 0. This reduced the oscillations at high speed tremendously.