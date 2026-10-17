"""
traction.py

Simulates the cornering force produced by the tires when the driver steers. The
difference between where the tires point (steer angle) and where the car is actually
moving is the slip angle. The tire produces a lateral force proportional to the slip
angle (cornering stiffness), which builds up lateral acceleration and lateral velocity.

In this test, the driver turns the steering from 0 to 5 degrees over 3 seconds, then holds 
it for 7 seconds. Steering angles are converted from degrees to radians before use.

Constants: 
Forward speed 15 m/s, cornering stiffness 36000 N/rad, mass 300 kg (car + driver). 
Lateral velocity starts at 0 m/s.
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from dataclasses import dataclass
import numpy as np


# --- Constants ---
FORWARD_SPEED        = 15.0      # m/s
CORNERING_STIFFNESS  = 36000.0   # N/rad
MASS                 = 300.0     # kg, car + driver
 
MAX_STEER_DEG = 5.0      # deg, final steering angle
RAMP_TIME     = 3.0      # s, steering goes 0 -> 5 deg
HOLD_TIME     = 7.0      # s, steering held at 5 deg
END_TIME      = RAMP_TIME + HOLD_TIME   # s, total simulation time
DT            = 0.01     # s, time step

def steer_input(t: float, max_steer_deg: float = MAX_STEER_DEG, ramp_time: float = RAMP_TIME) -> float:
    """
    Returns the driver's steering angle at a given time, converted to radians.
    Use this function to get the steering input at each time step of the simulation.

    Inputs:
    t: float, current simulation time (s)
    max_steer_deg: float, final steering angle (deg)
    ramp_time: float, time for the steering to go from 0 to max_steer_deg (s)

    Outputs:
    float, steering angle (rad); ramps linearly, then holds at max_steer_deg
    """
    steer_deg = max_steer_deg * np.clip(t / ramp_time, 0.0, 1.0)
    return float(np.radians(steer_deg))


def slip_angle(steer_angle: float, lat_velocity: float, forward_speed: float = FORWARD_SPEED) -> float:
    """
    Calculates the slip angle between where the tires point and where the car moves.
    Use this function after finding the steering angle to get the tire's slip angle.

    Inputs:
    steer_angle: float, steering angle (rad)
    lat_velocity: float, current lateral velocity (m/s)
    forward_speed: float, forward speed of the car (m/s)

    Outputs:
    float, slip angle (rad) = steer_angle - (lat_velocity / forward_speed)
    """
    return steer_angle - (lat_velocity / forward_speed)


def lateral_force(slip: float, cornering_stiffness: float = CORNERING_STIFFNESS) -> float:
    """
    Calculates the cornering force the tires produce from the slip angle.
    Use this function after finding slip angle to get the force pushing the car into the turn.

    Inputs:
    slip: float, slip angle (rad)
    cornering_stiffness: float, tire cornering stiffness (N/rad)

    Outputs:
    float, lateral force (N) = cornering_stiffness * slip
    """
    return cornering_stiffness * slip


def lateral_acceleration(force: float, mass: float = MASS) -> float:
    """
    Calculates the car's lateral acceleration from the lateral force using F = m * a.
    Use this function after finding lateral force to get how quickly lateral velocity builds up.

    Inputs:
    force: float, lateral force (N)
    mass: float, mass of the car with driver (kg)

    Outputs:
    float, lateral acceleration (m/s^2)
    """
    return force / mass


def lateral_velocity(prev_lat_velocity: float, lat_accel: float, dt: float = DT) -> float:
    """
    Updates the car's lateral velocity for the next time step.
    Use this function after finding lateral acceleration to step lateral velocity forward in time.

    Inputs:
    prev_lat_velocity: float, lateral velocity from the previous time step (m/s)
    lat_accel: float, lateral acceleration (m/s^2)
    dt: float, time step (s)

    Outputs:
    float, new lateral velocity (m/s) = prev_lat_velocity + (lat_accel * dt)
    """
    return prev_lat_velocity + (lat_accel * dt)


def traction_step(t: float, prev_lat_velocity: float, dt: float = DT) -> dict:
    """
    Runs the full traction chain for one time step:
    steer angle -> slip angle -> lateral force -> lateral acceleration -> lateral velocity.
    Use this function at each time step to get all traction values at once.

    Inputs:
    t: float, current simulation time (s)
    prev_lat_velocity: float, lateral velocity from the previous time step (m/s)

    Outputs:
    dict with keys "steer", "slip", "force", "a_lat", "v_lat"
    """
    steer = steer_input(t)
    slip  = slip_angle(steer, prev_lat_velocity)
    force = lateral_force(slip)
    a_lat = lateral_acceleration(force)
    v_lat = lateral_velocity(prev_lat_velocity, a_lat, dt)
    return {"steer": steer, "slip": slip, "force": force, "a_lat": a_lat, "v_lat": v_lat}


@dataclass
class State:
    t: float = 0.0       # s, time
    v_lat: float = 0.0   # m/s, lateral velocity (starts at 0)
 
def step (state:State) -> State:
    steer = steer_input(state.t)
    slip = slip_angle(steer, state.v_lat)
    force = lateral_force(slip)
    a_lat = lateral_acceleration(force)
    v_lat = lateral_velocity(state.v_lat, a_lat)
    return State(t=state.t + DT, v_lat=v_lat)

state = State()
times = []
lat_velocities = []
 
def animate (i):
    global state
    if state.t < END_TIME:   # stop once we reach 10 seconds
        state = step(state)
        times.append(state.t)
        lat_velocities.append(state.v_lat)
        line.set_data(times, lat_velocities)
    return line,
 
 
fig = plt.figure(figsize=(3,3), dpi=150)
ax = fig.add_subplot(111)
ax.grid()
ax.set_xlim(0, 10)
ax.set_ylim(0, 1.5)
# these lines are so the animation doesnt zoom in or out
ax.set_xlabel("time (s)")
ax.set_ylabel("lateral velocity (m/s)")
line, = ax.plot([], [])
plt.tight_layout()
plt.pause(3)
ani = animation.FuncAnimation(fig, animate, interval=0)
fig.canvas.draw()
plt.show()