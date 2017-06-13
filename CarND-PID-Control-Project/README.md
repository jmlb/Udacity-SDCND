# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid K_p K_d_ K_i min_thr`. 
[K_p K_d_ K_i] the 3 PID gains
min_thr the minimum throttle allowed : the throttle is restricted to the range [0.3, 0.8]

## Discussion
[watch the video](https://youtu.be/_wUoVPyukuI)

### Description of how the final hyperparameters were chosen.
I used both visualization and the not-normalized **rms error** to find the optimum PID gains. The PID gains are adjusted to reduce the not-normalized **rms error** accumulated over the course of a full lap completed by the agent.

$$ rms = sum of cte^2 $$

Below are the optimized PID gains value:

 Kp = 0.032, K_d = 0.006, K_i=1.3

And how to run the PID model:

 `./pid 0.032 0.006 1.3 0.3`

The last number (0.3) is the minimum threshold allowed. The throttle opening is limited to the range [0.3, 0.8] 

### Description of the effect each of the P, I, D components on the implementation.

+ P gain: controls how fast the agent reacts to error. With a high K_p, the agent reacts fast but result in an oscillatory trajectory of the car around the truth value of the cross track position.

+ I gain: enables to damp the oscillatory behavior. With an optimum I and P the car is able to converge towards the true cross track position (resulting in cte =0). If K_i is too small, it will take more oscillations before the agent stabilizes. If K_i is too large, the *cte* could continuously increase, and the car would get further away from the true cross track position.

+ D gain: K_d enables to compensate for the drift of the steering. A large K_d would make it harder for the car to turn left or right and reduce the cte.

My strategy was to first optimize K_p and K_i simulatenously while keeping K_d=0 until the trajectory of the car was satisfactory. By changing K_p and K_i simulatenously, we prevent the system to be trapped in a local minimum. Then, K_d was varied to further improve the car behavior in autonomous mode. 

This first optimization was conducted at constant throttle: 0.3. 
If the throttle is increased, the car would show more oscillatory behavior. For the car to drive faster, 
without sacrificing on the overall behavior on the road, we implement a small decision tree using if/else statements.
For example, if the accumulated cte (in absolute value) over the last 10 frames are below a threshold, we can assume 
that the car is "stable", and therefore increase the throttle. 
When the car starts to be unstable, i.e it shows significant oscillations, the throttle is reduced. 
          


