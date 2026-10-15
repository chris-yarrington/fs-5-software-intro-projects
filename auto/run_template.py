"""
Run script for the PID controller

Simulates the car for (STEPS) time steps. At each step:
    1. measure_velocity simulates a speed sensor (true velocity + random noise)
    2. calculate_desired_acceleration finds the velocity error and acceleration needed to correct it
       also low-pass filters the derivative so it doesn't amplify sensor noise
    3. acceleration_to_throttle_percentage converts the desired acceleration to a throttle command
    4. update applies the throttle and advances the car's state by one time step (dt),
       using the friction value scheduled for the current time

The friction changes in intervals (FRICTION_SCHEDULE) to act as a disturbance. The controller 
sees the velocity error that the friction causes, and the integral term adjusts to cancel it.
Plots velocity, error and friction over time."""

import numpy as np
import matplotlib.pyplot as plt
from pid_template import make_car
from pid_template import update
from pid_template import measure_velocity 
from pid_template import calculate_desired_acceleration
from pid_template import acceleration_to_throttle_percentage


# --- Constants ---

# Controller gains
K_P = 10.0  # proportional: reacts to the current error
K_I = 2.0   # integral: accumulates past errors
K_D = 0.2   # derivative: predicts future errors based on current rate of change

# Simulation settings
DESIRED_V = 20.0 # target velocity in m/s
DT = 0.1         # time step in s
STEPS = 550      # STEPS * DT = total simulation time in s

# Sensor settings
NOISE_STD = 0.1     # sensor noise (standard deviation) in m/s; 0 = perfect sensor
D_FILTER_TAU = 0.5  # derivative low-pass filter time constant in s; 0 = no filter
SEED = 0            # random seed, used to normalize testing

# Friction schedule: (start time in s, friction in m/s^2). Each value lasts until the next start time.
FRICTION_SCHEDULE = [
    (0.0, 2.0),   # normal road
    (15.0, 3.5),  # increased friction (e.g. rough surface)
    (25.0, 1.0),  # decreased friction (e.g. smooth surface)
    (35.0, 4.0),  # large increase
    (45.0, 2.0),  # back to normal
]



# --- Helper functions ---

def friction_at(t: float) -> float:
    """
    Returns the friction scheduled for time t.

    Inputs:
    t: float, current simulation time in s

    Outputs:
    friction: float, friction deceleration in m/s^2 from the latest schedule entry that has started
    """
    friction = FRICTION_SCHEDULE[0][1]
    for start_time, value in FRICTION_SCHEDULE:
        if t >= start_time:
            friction = value
    return friction



# --- Simulation ---

car = make_car(desired_v=DESIRED_V, dt=DT)
rng = np.random.default_rng(SEED)

velocities = []
measured_velocities = []
errors = []
throttles = []
times = []
frictions = []

for _ in range(STEPS):
    # Sensor: measure velocity with noise
    measured_v = measure_velocity(car, NOISE_STD, rng)

    # Controller: decide how hard to accelerate, then convert to throttle command
    acceleration_desired, error = calculate_desired_acceleration(car, K_P, K_I, K_D,
                                                                 measured_v=measured_v, d_filter_tau=D_FILTER_TAU)
    throttle_perc = acceleration_to_throttle_percentage(acceleration_desired)

    # get this step's friction
    friction = friction_at(car["t"])

    # record before update so time, velocities, error, and friction all describe same instant
    times.append(car["t"])
    velocities.append(car["v"])
    measured_velocities.append(measured_v)
    errors.append(error)
    throttles.append(throttle_perc * 100)  # convert to percentage for plotting
    frictions.append(friction)

    # apply throttle, advance by one time step
    update(car, throttle_perc, friction=friction)

# print check of where the controller settled
print("Final velocity:", round(velocities[-1], 4))
print("Final error:", round(errors[-1], 4))



# --- Plotting ---

plt.figure(figsize=(8, 10))

# Velocity plot: velocity should rise and level off at the dashed target line, with slight bumps at each friction change
plt.subplot(4, 1, 1)
plt.plot(times, velocities, label="velocity")
plt.axhline(DESIRED_V, linestyle="--", color="gray", label="desired velocity")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity over Time")
plt.legend()
plt.grid(True)

# Error plot: error should start at desired_v, decay toward the dashed zero line, and recover after friction changes 
plt.subplot(4, 1, 2)
plt.plot(times, errors, color="tab:red")
plt.axhline(0, linestyle="--", color="gray")
plt.ylabel("Error (m/s)")
plt.title("Error over Time")
plt.grid(True)

# Throttle plot: shows how much the controller passes sensor noise through to motor
plt.subplot(4, 1, 3)
plt.step(times, throttles, color="tab:purple", linewidth=0.8)
plt.ylabel("Throttle (%)")
plt.title("Throttle over Time")
plt.grid(True)

# Friction plot: friction schedule, drawn as steps since it changes instantly at each interval
plt.subplot(4, 1, 4)
plt.step(times, frictions, where="post", color="tab:green")
plt.xlabel("Time (s)")
plt.ylabel("Friction (m/s^2)")
plt.title("Friction over Time")
plt.grid(True)

plt.tight_layout()  # keep titles and labels from overlapping
plt.show()
