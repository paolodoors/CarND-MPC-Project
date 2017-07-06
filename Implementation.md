# Implementation

## The model

MPC is and advanced control method. The idea of this project is to control the car trajectory given a reference trajectory.

Simply put, MPC computes several steps ahead in time on where the car should be and how the actuators should be controlled to achieve this. But even though the trajectory is computed in advance, only the most recent step is taken into account to actuate the car and the process is repeated. This way the trajectory is always adapting to the new state of the car.

To achieve this we have to track the car state:

```
px  # position in X
py  # position in Y
psi # car's heading direction
v   # velocity
cte # cross track error
epsi# error psi
```
and the actuators:

```
delta   # steering angle
a       # throttle (or acceleration)
```

There are also several constraints on how the car state evolves over time (it'll neve turn 180° for example)
```cpp
x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
psi1 - (psi0 + v0 * delta0 / Lf * dt);
v1 - (v0 + a0 * dt);
cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```

where ```dt``` is the elapsed duration of each step, ```Lf``` is the distance between the front of the vehicle and its center of gravity, and ```psides``` is the desired car orientation.

Then we have the cost function, which accounts for the difference between the projected state and the desired state (or trajectory) and we want to minimize. Basically it is the square of the errors plus the square of the actuators values plus the square of the difference between states.

```cpp
// The part of the cost based on the reference state.
for (t = 0; t < N; t++) {
    fg[0] += CppAD::pow(vars[cte_start + t] - ref_cte, 2);
    fg[0] += CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
    fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
}

// Minimize the use of actuators.
for (t = 0; t < N - 1; t++) {
    fg[0] += CppAD::pow(vars[delta_start + t], 2);
    fg[0] += CppAD::pow(vars[a_start + t], 2);
}

// Minimize the value gap between sequential actuations.
for (t = 0; t < N - 2; t++) {
    fg[0] += 500 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
    fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}
```

There's a 500 multiplier to avoid making big turns between steps (like trying to turn 90° or something like that).

## Timestep Length and Elapsed Duration

Here I had one big thing in mind: try to avoid computation at all cost.

To meet this I thouth that ```dt``` should be set to multiple to the minimum latency. So I set ```dt``` to 0.1 (100ms latency) and then played with N. We'll talk more about this in the last section.

I started with 10 points, that did quiet well, the prediected trajectory followed the reference well, but sometimes it was a little wooby. So I started to step down until 6 that was the magic number where the car followed the reference very well with almost no sinusoidal moves. (with just 5 step the car couldn't follow the reference at all).

In code this is set at:
```
size_t N = 6;
double dt = 0.1;
```

## Polynomial Fitting and MPC Preprocessing

Given that the simulator provides the coordinates in a global reference system I've changed that to the car's own reference system to ease the computation.

This is done at:

```cpp
for (int i = 0; i < pts_size; i++) {
    double shift_x = ptsx[i] - px;
    double shift_y = ptsy[i] - py;
    waypoints_x(i) = sin(psi) * (shift_y) + cos(psi) * (shift_x);
    waypoints_y(i) = cos(psi) * (shift_y) - sin(psi) * (shift_x);
}
```

## Dealing with latency

The model should be able to deal with a 100ms latency. To overcome this issue we have to estimate where the car will be after those 100ms and since I've used ```dt = 0.1```, that value was the first prediction step.

In code these are the lines:
```python
result.push_back(solution.x[delta_start]);
result.push_back(solution.x[a_start]);
```