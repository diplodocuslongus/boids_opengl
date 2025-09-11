# Boids

Implementation of Craig Reynolds' boids, which simulate the flocking behaviour of birds.

*The basic flocking model consists of three simple steering behaviors which describe how an individual boid maneuvers based on the positions and velocities its nearby flockmates:*
 - *Separation: steer to avoid crowding local flockmates*
- *Alignment: steer towards the average heading of local flockmates*
- *Cohesion: steer to move toward the average position of local flockmates*

(See https://www.red3d.com/cwr/boids/)

I used the *Dear ImGui* library for the graphical user interface (See https://github.com/ocornut/imgui).

## 0 - Modifications (diplodocuslongus)

- CMakeLists modif so the code runs on ubuntu 22/24
- fix zoom in out with mouse wheel scroll
- Command line argument inputs

## 1 - Installation

Dependencies (tested on popos24 / ubuntu 24)

    sudo apt install libglew-dev
    sudo apt install libgl1-mesa-dev
    sudo apt install libeigen3-dev
    sudo apt install freeglut3-dev


First, clone the repository.
```
git clone https://github.com/ThomasParistech/boids.git
```
Then, go to the directory and compile it.
```
cd boids
mkdir build
cd build
cmake ..
make -j6
```
Please note that CMakeLists.txt is configured in a such way that the executable is generated in the "bin" directory.

## 2 - Running

Go to the build directory and run the app
```
bin/main
```

![](./images/spiral8.gif)

## 3 - Flocking behavior

Alignment rule:
![](./images/alignment.gif)

Cohesion rule:
![](./images/cohesion_very_short.gif)

Object attirance/avoidance rule:
![](./images/holecross2.gif)
![](./images/hole_boid.gif)

## simulation parameters

Most are self explanatory.
<!--  -->
neighborhood_max_disti: defines the 

separation_min_dist_ = 1;

separation_factor_ = 0.02;
cohesion_factor_ = 0.07;
alignment_factor_ = 0.005;

randomness_ = 0;
max_speed_ = 10.f;

min_cos_angle_ = -0.5f;

next_id_ = 0;
