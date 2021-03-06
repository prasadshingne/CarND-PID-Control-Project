# CarND-Controls-PID

Self-Driving Car Engineer Nanodegree Program ([Udacity Starter Code](https://github.com/udacity/CarND-PID-Control-Project))

In this project we revisit the lake race track from the Behavioral Cloning Project. This time, however, we implement a PID controller in C++ to maneuver the vehicle around the track!

The simulator provides the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle and throttle position. The speed limit has been increased from 30 mph to 100 mph. **NOTE: We don't have to meet a minimum speed to pass.**

Use the Term 2 Simulator from [here](https://github.com/udacity/self-driving-car-sim/releases). This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


Once setup is complete build and run the code as follows:

1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./pid`. 

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.

## My Simulator and Environment Set-up (Ubuntu 20.04.1 LTS)

### Simulator Setup

The instructions are from [here](https://medium.com/@kaigo/how-to-install-udacitys-self-driving-car-simulator-on-ubuntu-20-04-14331806d6dd).

1. Download the (.deb) package of Unity (3D) version 5.5.1f1 that the Udacity Simulator uses. 
2. Install dependencies : `sudo apt install gconf-service lib32gcc1 lib32stdc++6 libc6-i386 libgconf-2-4 npm`
3. Run the install : `sudo dpkg -i ~/Downloads/unity-editor_amd64-5.5.1xf1Linux.deb`
4. If you get error about unmet dependencies you may need to run, and retry ` sudo apt --fix-broken install` 
5. With Unity working now, download and run the latest release of the Udacity Term 2 [Simulator](https://github.com/udacity/self-driving-car-sim/releases). 

### Environment Setup

I used VSCode IDE for this project. Follow the thorough instructions provided by [yosoufe](https://gist.github.com/yosoufe/dd37284b7319c484dd77e42947fc82b7) to setup the environment. The instructions cover debugging as well which maybe useful.


# [Rubric](https://review.udacity.com/#!/rubrics/1972/view)

## Compilation

My code compiles without errors with cmake and make. I didn't make any changes to CMakeLists.txt.

Output: cmake
<img src="https://github.com/prasadshingne/CarND-PID-Control-Project/blob/master/output/cmake.jpg"/>

Output: make
<img src="https://github.com/prasadshingne/CarND-PID-Control-Project/blob/master/output/make.jpg"/>


## Implementation

I followed the lessons to implement PID control. I have no additional .cpp or .h files although I use two separate PIDs for steering and throttle control. The implementation is done in [./src/PID.cpp](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/master/src/PID.cpp).[PID::UpdateError](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/98764fad334ce323884c301b6c5c24a01679e90c/src/PID.cpp#L27) calculates the proportional, integral and derivative errors and [PID::TotalError](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/98764fad334ce323884c301b6c5c24a01679e90c/src/PID.cpp#L39) calculates the total error with the respective coefficients. I have another method [PID::TotalSpeedError](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/98764fad334ce323884c301b6c5c24a01679e90c/src/PID.cpp#L53) that does the same thing as [PID::TotalError](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/98764fad334ce323884c301b6c5c24a01679e90c/src/PID.cpp#L39) except that it limits the output to [0, 1] for the throttle instead of [-1, 1] as in the case of the steering.


## Reflection


### The effect each of the P, I, D components 
* The proportional control tries to reduce the error according to the control input ![formula](https://render.githubusercontent.com/render/math?math=\color{gray}%20{U_{P}(t)%20=%20-%20K_{P}(t)%20\cdot%20CTE(t)}%20). If this is used on its own the car oscillates around the center of the road and the overshoots eventually drive it out of the road as seen in the gif below.

<img src="https://github.com/prasadshingne/CarND-PID-Control-Project/blob/master/output/P.gif"/>

* The integeral control tries to reduce the integerated error according to control input ![formula](https://render.githubusercontent.com/render/math?math=\color{gray}%20{U_{I}(t)%20=%20-%20K_{I}(t)%20\cdot%20\int_{0}^{t}%20CTE(t)}). Integral control is ususlly not applied on its own and its main use is reducing the offset error after P and D are applied. I show below the result for PI control where ![formula](https://render.githubusercontent.com/render/math?math=\color{gray}%20{U_{PI}(t)%20=%20K_{P}(t)%20%2B%20K_{I}(t)}). As you can see even the PI controller is not capable to produce stable result. The car overshoots the lane center and leaves the road.

<img src="https://github.com/prasadshingne/CarND-PID-Control-Project/blob/master/output/PI.gif"/>

* The differential control tries to reduce the differential error according to the control input ![formula](https://render.githubusercontent.com/render/math?math=\color{gray}%20{U_{D}(t)%20=%20-%20K_{D}(t)%20\cdot%20\frac{\text{d}CTE(t)}{\text{d}t}}). Differential control is also not typically applied on its own and its main use is as a damper to reduce the oscillations and reduce the settling time for a second order system. Below I compare the PD (![formula](https://render.githubusercontent.com/render/math?math=\color{gray}%20{U_{PD}(t)%20=%20U_{P}%20%2B%20U_{D}})) and PID (![formula](https://render.githubusercontent.com/render/math?math=\color{gray}%20{U_{PID}(t)%20=%20U_{P}%20%2B%20U_{I}%20%2B%20U_{D}})) control. You will notice that both controllers work to keep the car within the lanes. The car with PD control takes a wider turn and PID control keeps the car closer to the center even in the curve.


|  PD    |    PID    |
|:------:|:---------:|
|<img src="https://github.com/prasadshingne/CarND-PID-Control-Project/blob/master/output/PD.gif"/>|<img src="https://github.com/prasadshingne/CarND-PID-Control-Project/blob/master/output/PID.gif"/>|

### Choosing the final hyperparameters

For the throttle control, I assumed that the speed limit for the track should be 35 mph, hence my target speed was [33 mph](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/c72400acc1909a6133a478c3b3ee599b49bb4db6/src/main.cpp#L99). I selected the PID gains [([Kp, Ki, Kd] = [0.025, 0.0001, 2.5])](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/c72400acc1909a6133a478c3b3ee599b49bb4db6/src/main.cpp#L91) for throttle contrller completely manually and the car seems to drive at 33 +/- 1 mph. The tuning for the steering control was much more involved. I first arrived at an initial guess for the optimization and the bounding constrants on the PID gains manually. I used Python and [Scipy optimizer 'minimize'](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html#scipy.optimize.minimize) to minimize the root-mean-squared (RMS) cross track error. Please refer to the implementation in [../python/optimize_pid_steer.py](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/master/python/optimize_pid_steer.py). 

Before getting into the optimizer, I modified the main.cpp to include a [resetSimulator(uWS::WebSocket<uWS::SERVER> ws)](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/c72400acc1909a6133a478c3b3ee599b49bb4db6/src/main.cpp#L39) function and added a variable [sim_dur](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/c72400acc1909a6133a478c3b3ee599b49bb4db6/src/main.cpp#L102) to set the time to simulate each execution during optimization. If the simulation time is greater than the value of sim_dur I [reset the simulation](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/c72400acc1909a6133a478c3b3ee599b49bb4db6/src/main.cpp#L156). In order to iterate the execution I also added code to [read the PID gains for this iteration](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/c72400acc1909a6133a478c3b3ee599b49bb4db6/src/main.cpp#L58) and to [write several outputs](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/c72400acc1909a6133a478c3b3ee599b49bb4db6/src/main.cpp#L149), especially the CTE which is to be minimized.

The script [optimize_pid_steer.py](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/master/python/optimize_pid_steer.py) contains the function [run_pid(p)](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/c72400acc1909a6133a478c3b3ee599b49bb4db6/python/optimize_pid_steer.py#L32) which takes in the (normalized) PID gains as a Python list p, [writes the gains to the input file](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/c72400acc1909a6133a478c3b3ee599b49bb4db6/python/optimize_pid_steer.py#L47), executes [./pid](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/c72400acc1909a6133a478c3b3ee599b49bb4db6/python/optimize_pid_steer.py#L50), [reads the output file](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/c72400acc1909a6133a478c3b3ee599b49bb4db6/python/optimize_pid_steer.py#L53), [computes and returns the error](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/c72400acc1909a6133a478c3b3ee599b49bb4db6/python/optimize_pid_steer.py#L57) as the output. I use the optimizer method ['SLSQP'](https://docs.scipy.org/doc/scipy/reference/optimize.minimize-slsqp.html#optimize-minimize-slsqp) in minimize function. I [normalize](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/c72400acc1909a6133a478c3b3ee599b49bb4db6/python/optimize_pid_steer.py#L89) the initial guess and the bounds to ensure that the optimizer changes the PID gains proportionally. The list (p) in the function run_pid(P) has to be ['denormalized'](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/c72400acc1909a6133a478c3b3ee599b49bb4db6/python/optimize_pid_steer.py#L42) before writing the input file and results have to be ['denormalized'](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/c72400acc1909a6133a478c3b3ee599b49bb4db6/python/optimize_pid_steer.py#L102). After running the optimizer for hours I got the optimized gains for the steering as [[Kp, Ki, Kd] = [0.14937642030249174, 0.0008028211368793284, 3.355222551300104]](https://github.com/prasadshingne/CarND-PID-Control-Project/blob/c72400acc1909a6133a478c3b3ee599b49bb4db6/src/main.cpp#L80).

Note: I acknowledge that the tuning of the PID could be done in multiple different ways and I could have spent much more time on this, playing with the simulation duration, using cumulative error CTE than RMS error CTE. Further, there is a possibility of using a weighted sum of the steering error and the speed error to optimize parameters for both the throttle PID and steer PID together. I made an assumption about the speed limit. Another control law altogether may be used for the throttle perhaps dependent on the CTE.

## Simulation

The vehicle successfully drives a lap around the track with my selected PID gain values. My solution video is shared via YouTube. Link below.

<a href="https://www.youtube.com/watch?v=5AQ89yUmonM" target="_blank"><img src="http://img.youtube.com/vi/5AQ89yUmonM/0.jpg" alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>



