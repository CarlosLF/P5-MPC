# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


[UpdateEqs]: ./update_eqs.png "Update Equations"

## The Model
### Student describes their model in detail. This includes the state, actuators and update equations.

As we read in lecture 19, the model used in this project is a kinematic bicycle model. The state of the of the vehicle includes 

1. px, position of the vehicle in the X axis
2. py, position of the vehicle in the Y axis
3. psi, vehicle's orientation
4. v, Velocity of the vehicle
5. cte, Cross-track error
6. epsi,  Orientation error

The control inputs are

1. Acceleration
2. Steering angle


The update equations are the following

![alt text][UpdateEqs]



## Timestep Length and Elapsed Duration (N & dt)

### Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

T is the prediction horizon is the duration over which future predictions are made. The variable T is defined by the product of two other variables, N and dt.
Where N is the number of timesteps in the horizon, and dt is how much time elapses between actuations. 

In this project I test the following pair of parameters 

1. N=20, dt=0.1, 
In this case the prediction horizon is large, and the car behaivor is well when the road is straight, or when the turns are small. However when the turns are complicated the vehicle begins to oscillate and gets off the track.

2. N=20, dt=0.5,
In this case the dt was reduced by a half, and so the value of the horizon T. Whit this change the vehicle never gets off the road, and it follows the referece well.

3. N=12, dt=0.5,
In this case the number of timessteps in the horizon was reduced, and as the lecture 20.5 says "Beyond that horizon, the environment will change enough that it won't make sense to predict any further into the future." This pair is the option that I choose, since the vehicle's behavior is well on straight and turns, and it never gets off the track.

An example of the MPC controller results are shown in [MPC Controller Video](
https://www.dropbox.com/s/151ehq14vydlnys/mpc.mov?dl=0)

## Polynomial Fitting and MPC Preprocessing


The computation were performed in the coordinate system of the vehicle. Therefore, the waypoints coordinates are transformed into the coordinates of the vehicle. This is acomplish by using a 2D rigid transformation, where the translation is defined by the current pose of the vehicle, and a rotation. Once that the waypoints are defined in the coordinates of the vehicle, we fit a third order polynomial to the waypoints. 

This transformation was performed with this code

```cpp

  for (int i = 0; i< ptsx.size(); i++){

      // apply translation to (px, py)
      double shift_x = ptsx[i] - px;
      double shift_y = ptsy[i] - py;
      
      // apply rotatation with -psi angle
      ptsx[i] = shift_x * cos(-psi) - shift_y * sin(-psi);
      ptsy[i] = shift_x * sin(-psi) + shift_y * cos(-psi);
  }

```

## Model Predictive Control with Latency

### The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

To deal with latency we use what we learn in lecture 20.7

"A contributing factor to latency is actuator dynamics. For example the time elapsed between when you command a steering angle to when that angle is actually achieved. This could easily be modeled by a simple dynamic system and incorporated into the vehicle model. One approach would be running a simulation using the vehicle model starting from the current state for the duration of the latency. The resulting state from the simulation is the new initial state for MPC."

Thus to deal with latency we use the kinematics model of the vehicle, the code used in the project was 

```cpp
//Using the model defined in lecture 19.4
double x0 = 0.0;
double y0 = 0.0;
double psi0 = 0.0;
double v0 = v;

if (time_latency > 0){
    double dt = time_latency / 1000.0; // latency in seconds
    // In the car coordinates psi = 0
    x0 = v * dt;
    
    psi0 =  - (v/Lf) * delta_t * dt ;
    v0 = v + a_t * dt;
    
    //Cross Track Error, lecture 19.10
    cte += v * sin(epsi) * dt;
    
    //Orientation error, lecture 19.10
    epsi -= (v/Lf) * delta_t * dt ;
}

```