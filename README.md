[//]: # (Image References)

[img1]: ./images/MPC_scheme_basic.svg "MPC_scheme_basic"
[img2]: ./images/equations.png "equations"
[img3]: ./images/state.png "state"
[img4]: ./images/example.png "example"
___
# SDCND Term 2 Project 10: Model Predictive Control
## Model Predictive Control Project for Udacity's Self-Driving Car Engineer Nanodegree Program
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This project involves the implementation of a Model Predictive Controller in C++ to control a vehicle in the [simulator of Udacity](https://github.com/udacity/self-driving-car-sim/releases). The simulator sends global target points x and y (yellow in the video) to the controller via WebSocket and receives the steering signal and throttle value ([-1, 1] normalized). [Here](https://github.com/udacity/CarND-MPC-Project) you can find the seed project of Udacity.

#### The results can be viewed here(Youtube):
[![result1](https://img.youtube.com/vi/02ajYi8aZDI/0.jpg)](https://www.youtube.com/watch?v=02ajYi8aZDI)

## Implementation

### Overview
Model predictive control (MPC) is an advanced method of process control that is used to control a process while satisfying a set of constraints. Model predictive controllers rely on dynamic models of the process, most often linear empirical models obtained by system identification. The models used in MPC are generally intended to represent the behavior of complex dynamical systems. The additional complexity of the MPC control algorithm is not generally needed to provide adequate control of simple systems, which are often controlled well by generic PID controllers. Common dynamic characteristics that are difficult for PID controllers include large time delays and high-order dynamics. [source](https://en.wikipedia.org/wiki/Model_predictive_control)

![img1]

MPC is based on iterative, finite-horizon optimization of a plant model (in this case vehicle model). At time **t** the current vehicle state is sampled and a cost minimizing control strategy is computed (via a numerical minimization algorithm) for a time horizon in the future **t + dt**. This algorithm also handles a set of constraints, which in this case model the motion physics of the vehicle.


### Detailed
#### Vehicle state
The state of the vehicle can be described with a vector of 4 elements:
* X-Position: x
* Y-Position: y
* Vehicle Angle: psi
* Vehicle speed: v

These 2 additional elements are needed for the controller:
* Cross Track Error: cte
* Error angle between target trajectory and vehicle Psi: err_psi


![img3]

#### Simulation telemetry
The following signals can be obtained from the simulation:
* ``targetPoints_x = j[1]["ptsx"];``
* ``targetPoints_y = j[1]["ptsy"];``
* ``posGlob_x = j[1]["x"];     // position x``
* ``posGlob_y = j[1]["y"];     // position y``
* ``psi = j[1]["psi"];``
* ``v = j[1]["speed"];``
* ``delta = j[1]["steering_angle"];``
* ``throttle_value  = j[1]["throttle"];``

The ``targetPoints`` are transformed to the vehicle coordinate system, this is done in ``main.cpp `` lines **151-159**. These target points are then used for a 3rd order polyfitting, result are the coefficients of this polynom. With the following equations the current ``cte`` and ``err_psi`` can be calculated:

``cte = coeffs[0];``
``err_psi = -atan(coeffs[1]);``

After this the delay compensation will be calculated, read below for more.

The following image is an edited image from udacity's course on MPC, which hopefully clarifies the subject a bit.
![img4]

#### Model
Futhermore the physical vehicle model is described by these equations:

![img2]

These equations are reshaped to zero to be useable for the optimizer. You can find this in ``MPC.cpp`` lines **111-117**.



#### Delay, Optimization
Every real system has some kind of delay between command and action, to simulate this a artificial delay of 100ms is implemented in ``main.cpp``. To compensate for this delay in the controller, the equations above are used to predict the state of the vehicle after the delay. Here psi is 0, so the equations simplify to what is written in ``main.cpp`` lines **169-174**.  If this delay is small enough this works, huge delays(>500ms) were tested but resulted in no working controller.

This predicted state together with a cost function and the constraints is then fed into the Ipopt-optimizer, to optimize the next **N** actions (steering and throttle).


#### Cost function
Custom cost functions were implemented to allow safe and fast driving. The following exemplary cost function dictates the optimizer to find a comprise between speed and the amount of steering.

``fg[0] += CppAD::pow(vars[v_start + t], 2) * CppAD::pow(vars[delta_start + t] * factor_DeltaAndSpeed, 2);``

So if  ``factor_DeltaAndSpeed`` is set with a high value, the controller drives slowly in corners, where a lot of steering has to happen to fulfill the other cost functions, which can be found in ``MPC.cpp`` lines **56-73**. Each cost function has its own factor to weight in the effect each function has.

 ``factor_DeltaAndSpeed`` can be regarded as a "caution value". The smaller the value, the more dangerous the vehicle drives.

#### Timestep Length and Elapsed Duration (N & dt) and other parameter
Different values were tried for N and dt. The following values have proven to be generally usable: ``N = 7`` and ``dt = 0.1``. ``N = 7`` is relatively small, but this results in less computation and works nevertheless. **ALL** parameter are loaded with the initialization of the MPC from the file ``paramater.dat`` in the order `` N, dt, ref_v, factorCTE, factorErrorPsi, factorErrorV, factor_DeltaAndSpeed, factor_d_Delta, factor_d_Thrust``.


## Comments
* There are two parameter sets. ``parameter.dat`` is for normal relatively safe driving. ``parameter_highspeed.dat`` was tuned to get the highest  speed possible, which is around **112.5 mph**. To use this parameter set, rename the old ``parameter.dat`` to ``parameter2.dat`` or something like this. Then rename ``parameter_highspeed.dat`` to ``parameter.dat``. With the next ``mpc`` call, the new parameter will be executed.



## Dependencies

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* Udacity's simulator

## Setup and Running
These are the suggested steps for Windows setup:

* Follow these [instructions](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) for setting up Ubuntu BASH.
* Follow these [instructions](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/install_Ipopt_CppAD.md) for installing Ipopt and CppAD
* Download Windows simulator [here](https://github.com/udacity/self-driving-car-sim/releases).
* Open Ubuntu Bash (write following commands to Ubuntu Bash command window)
* ``sudo apt-get update``
* ``sudo apt-get install git``
* ``sudo apt-get install cmake``
* ``sudo apt-get install openssl``
* ``sudo apt-get install libssl-dev``
* navigate to where you want to clone this repository to, for example:
 ``cd /mnt/c/Users/Bob``
* ``git clone https://github.com/autonomobil/SDCND-P10_Model-Predictive-Control``
* navigate to project folder: ``cd SDCND-P9_PID-Control``
* ``sudo rm /usr/lib/libuWS.so``
* ``./install_ipopt.sh``
* ``./install-ubuntu.sh``
* navigate to the build folder: ``cd build``
* Execute ``cmake .. && make``
* Launch the **term2_sim.exe** from Windows simulator folder
* Execute ``./mpc``
* If you see ``Listening to port 4567 Connected!!!``, it is working
* Press **Start**


All C++ files were modified compared to the [original repository](https://github.com/udacity/CarND-MPC-Project):  
