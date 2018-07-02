# Project Rubic Discussions

## 1. The Model - Student describes their model in detail. This includes the state, actuators and update equations

The vehicle model used in this project was derived in the class. 

The parameters to define the vehicle state are ![img](https://latex.codecogs.com/gif.latex?%5Cinline%20x%2C%5C%20y%5C%20%28positions%29%2C%20%5C%20%5Cpsi%5C%20%28heading%29%2C%20%5C%20v%5C%20%28velocity%29), and 2 actuators that control the car are ![img](https://latex.codecogs.com/gif.latex?%5Cinline%20%5Cdelta%5C%20%28steering%29%2C%5C%20a%5C%20%28throttle%29).

To update the state from `t` to `t+1`, I used the following equations derived in the class:

![img](https://latex.codecogs.com/gif.latex?%5C%5Cx_%7Bt&plus;1%7D%3Dx_%7Bt%7D&plus;v_%7Bt%7Dcos%28%5Cpsi_%7Bt%7D%29*dt%20%5C%5Cy_%7Bt&plus;1%7D%3Dy_%7Bt%7D&plus;v_%7Bt%7Dsin%28%5Cpsi_%7Bt%7D%29*dt%20%5C%5C%5Cpsi_%7Bt&plus;1%7D%3D%5Cpsi_%7Bt%7D&plus;%5Cfrac%7Bv_%7Bt%7D%7D%7BL_%7Bf%7D%7D%5Cdelta_%7Bt%7D*dt%20%5C%5Cv_%7Bt&plus;1%7D%3Dv_%7Bt%7D&plus;a_%7Bt%7D*dt%20%5C%5Ccte_%7Bt&plus;1%7D%3Df%28x_%7Bt%7D%29-y_%7Bt%7D&plus;%28v_%7Bt%7Dsin%28e%5Cpsi_%7Bt%7D%29dt%29%20%5C%5Ce%5Cpsi_%7Bt&plus;1%7D%3D%5Cpsi_%7Bt%7D-%5Cpsi%20des_%7Bt%7D&plus;%28%5Cfrac%7Bv_%7Bt%7D%7D%7BL_%7Bf%7D%7D%5Cdelta_%7Bt%7D*dt%29)

From `Line 87 - 121` in the `MPC.cpp` file, I used the above equations to setup constrains for each of the parameter by limiting their relationships between time t and t-1. These constrains along with cost function will be used in the `MPC::Solve` method to calculate the lowest cost.

The cost function is defined as follows. I first minimize the cte, epsi and velocity to pre-set reference (50 mph in my project). Then I also minimize the change rate for steering and throttle. I decided to add 10 times to throttle to make it more smooth. Lastly, I also minimize the change between sequential acuations. 500 factor has been applied to the steering term so that it would not abrutly turn the wheel to ensure safe driving.

```C++
// Minimize cte, epsi and velocity to reference
for (unsigned int t = 0; t < N; ++t) {
    fg[0] += CppAD::pow(vars[cte_start + t], 2);
    fg[0] += CppAD::pow(vars[epsi_start + t], 2);
    fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

// Minimize change rate
for (unsigned int t = 0; t < N - 1; ++t) {
    fg[0] += CppAD::pow(vars[delta_start + t], 2);
    fg[0] += 10 * CppAD::pow(vars[a_start + t], 2);        
}

// Minimize change between sequential acuations
for (unsigned int t = 0; t < N - 2; ++t) {
    fg[0] += 500 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
    fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);        
}
```

## 2. Timestep Length and Elapsed Duration (N & dt)

N & dt values were chosen as below. Ideally, the N should be as large while dt to be as small as possible to make the model prediction more accurate. But there would be a trade-off of the computing time and efficiency. I did not tried too many values and quickly found the ones that worked for me.

```C++
size_t N = 10;
double dt = 0.05;
```

I tried dt = 0.1 and dt = 0.05 and found the computing time is about instant and the accuracy is reasonably enough. As for N, I tried N = 25 (as in the quiz) and N = 10 and realized the N = 10 is sufficient for this project. Although the MPC controller predict N steps forward but it only adapt 1 step and re-calcualte for t+1. I am satisfied with N = 10 steps and it works great to plot the prediction line as well.

## 3. Polynomial Fitting and MPC Preprocessing

The vehicle state received from the simulator were first transformed from map's coordinates to car's coordinates by using the following equations.

![img](https://latex.codecogs.com/gif.latex?%5C%5Cx_%7Bcar%7D%3D%5C%20%5C%20cos%28%5Cpsi%29*dx_%7Bmap%7D%20&plus;%20sin%28%5Cpsi%29%20*%20dy_%7Bmap%7D%20%5C%5Cy_%7Bcar%7D%3D-sin%28%5Cpsi%29*dx_%7Bmap%7D%20&plus;%20cos%28%5Cpsi%29%20*%20dy_%7Bmap%7D)

Where dx and dy were delta between waypoints location to car location in map's coordinates and psi is the angle between two coordinates, ie car's heading.

The benefit to transform to car's coordinates is that we can then set x, y and heading of the vehicle to zeros. This will largedly simplify subsequent calculations.

The transformed state vectors were then fitted by the 3rd degree polynomial as it is sufficient for most of the road curvature. cte and epsi were then calculated by the vehicle model equations as memtioned in discussion 1.

## 4. Model Predictive Control with Latency

Before the transformed and fitted vehicle state were sent to MPC to solve the cost function. There is a 100 milisecond latency built-in in the simulator needs to be handled.

To account for this latency, I used the vehicle model equations to predict the vehicle state at 100 milisecond later. Those state parameters were then sent to MPC Solver to minimize the cost we defined earlier.

```C++
// Predict state after 100 milisecond latency
double dt_lat = 0.1;
const double Lf = 2.67;
px_c += v * cos(psi_c) * dt_lat;
py_c += v * sin(psi_c) * dt_lat;
psi_c += (v / Lf) * (- steer_value) * dt_lat;
v += throttle_value * dt_lat;
cte += v * sin(epsi) * dt_lat;
epsi += (v / Lf) * (- steer_value) * dt_lat;
```

# Original CarND-Controls-MPC Project README
Self-Driving Car Engineer Nanodegree Program

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
