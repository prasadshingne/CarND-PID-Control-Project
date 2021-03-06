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
1. The proportional control tries to reduce the error according to the control input <img src="https://render.githubusercontent.com/render/math?math=$U_{P}$(t) = -$K_{P}$(t) x CTE(t)">. If this is used on its own the car oscillates around the center of the road and the overshoots eventually drive it out of the road as seen in the gif below.

<img src="https://github.com/prasadshingne/CarND-PID-Control-Project/blob/master/output/P.gif"/>

2. The integeral control tries to reduce the integerated error according to control input <img src="https://render.githubusercontent.com/render/math?math=$U_{I}$(t) = -$K_{I}$(t) x $\int_{0}^{t}$ CTE(t)">. Integral control is ususlly not applied on its own and I show below the result for PI control where <img src="https://render.githubusercontent.com/render/math?math=$U_{PI}$(t) = $U_{P}$(t) + $U_{I}$(t)">. As you can see even the PI controller is not capable to produce stable result. The car overshoots the lane center and leaves the road.

<img src="https://github.com/prasadshingne/CarND-PID-Control-Project/blob/master/output/PI.gif"/>
