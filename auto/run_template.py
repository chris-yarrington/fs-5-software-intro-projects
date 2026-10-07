"""
Run script for the PID controller

Simulates the car for (STEPS) time steps. At each step:
    1. calculate_desired_acceleration finds the velocity error and acceleration needed to correct it
    2. acceleration_to_throttle_percentage converts the desired acceleration to a throttle command
    3. update applies the throttle and advances the car's state by one time step (dt)

Then takes velocity and error data and plots it over time. If the controller works, 
both curves should level off, with velocity near desired_v and error near 0."""


import matplotlib.pyplot as plt
from pid_template import make_car
from pid_template import update
from pid_template import calculate_desired_acceleration
from pid_template import acceleration_to_throttle_percentage

# Controller gains
K_P = 10.0
K_I = 2.0
K_D = 0.2
 
STEPS = 550 # STEPS * dt of 0.1s = STEPS/10 seconds of simulation time
 
car = make_car(desired_v=20.0, dt=0.1)


# --- Simulation ---

velocities = []
errors = []
times = []

for _ in range(STEPS):
    # Controller: decide how hard to accelerate, then convert to throttle command
    acceleration_desired, error = calculate_desired_acceleration(car, K_P, K_I, K_D)
    throttle_perc = acceleration_to_throttle_percentage(acceleration_desired)

    # record before update so time, velocities, and error all describe same instant
    times.append(car["t"])
    velocities.append(car["v"])
    errors.append(error)

    # apply throttle, advance by one time step
    update(car, throttle_perc)

# Numeric check of where the controller settled
print("Final velocity:", round(velocities[-1], 4))
print("Final error:", round(errors[-1], 4))


# --- Plotting ---

plt.figure(figsize=(8, 6))

# Velocity plot (2 rows, 1 column, position 1): elocity should rise and level off at (or near) the dashed target line
plt.subplot(2, 1, 1)
plt.plot(times, velocities, label="velocity")
plt.axhline(car["desired_v"], linestyle="--", color="gray", label="desired velocity")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity over Time")
plt.legend()
plt.grid(True)

# Bottom plot (2 rows, 1 column, position 2): error should start at desired_v and decay toward the dashed zero line
plt.subplot(2, 1, 2)
plt.plot(times, errors, color="tab:red")
plt.axhline(0, linestyle="--", color="gray")
plt.xlabel("Time (s)")
plt.ylabel("Error (m/s)")
plt.title("Error over Time")
plt.grid(True)

plt.tight_layout()  # keep titles and labels from overlapping
plt.show()