"""
throttle.py
 
Simulates how pressing the accelerator pedal turns into acceleration. The driver input
(normalized 0.0 to 1.0) is multiplied by the maximum motor torque to get command
torque, which is converted to force at the wheels through the gear ratio and wheel
radius. Acceleration is then found from F = m * a.
 
Constants: 
max motor torque 180 Nm, mass 300 kg (car + driver), gear ratio 3:1, wheel radius 0.216 m.
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from dataclasses import dataclass
import numpy as np


# --- Constants ---
MAX_TORQUE   = 180.0   # Nm, max motor torque
MASS         = 300.0   # kg, car + driver
GEAR_RATIO   = 3.0     # 3:1 reduction
WHEEL_RADIUS = 0.216   # m
 
RAMP_TIME = 7.0        # s, time from pedal = 0% to pedal = 100%
HOLD_TIME = 15.0       # s, time pedal held at 100%
END_TIME  = RAMP_TIME + HOLD_TIME   # s, total simulation time
DT        = 0.05                    # s, simulation time step
N_STEPS   = int(round(END_TIME / DT))


@dataclass
class State:
    t: float = 0.0         # s, simulation time
    x: float = 0.0         # m, distance traveled
    v: float = 0.0         # m/s, forward velocity
    a: float = 0.0         # m/s^2, forward acceleration
    throttle: float = 0.0  # pedal position (0.0 to 1.0)
    torque: float = 0.0    # Nm, command torque
    force: float = 0.0     # N, force at the wheels


def driver_input(t: float, ramp_time: float = RAMP_TIME) -> float:
    """
    Returns the driver's normalized pedal position at a given time.
    Use this function to get the throttle input at each time step of the simulation.
 
    Inputs:
    t: float, current simulation time (s)
    ramp_time: float, time for the pedal to go from 0.0 to 1.0 (s)
 
    Outputs:
    float, pedal position (0.0 to 1.0); ramps linearly, then holds at 1.0
    """
    return float(np.clip(t / ramp_time, 0.0, 1.0))


def command_torque(throttle: float, max_torque: float = MAX_TORQUE) -> float:
    """
    Calculates the command torque (rotational force needed from the motor).
    Use this function after finding the driver input to get the torque requested from the motor.
 
    Inputs:
    throttle: float, pedal position (0.0 to 1.0)
    max_torque: float, maximum motor torque (Nm)
 
    Outputs:
    float, command torque (Nm)
    """
    return throttle * max_torque
 
 
def wheel_force(torque: float, gear_ratio: float = GEAR_RATIO, wheel_radius: float = WHEEL_RADIUS) -> float:
    """
    Calculates the force at the wheels from the command torque.
    Use this function after finding command torque to get the force that pushes the car forward.
 
    Inputs:
    torque: float, command torque (Nm)
    gear_ratio: float, gear reduction ratio (3:1)
    wheel_radius: float, radius of the wheels (m)
 
    Outputs:
    float, force at the wheels (N) = (torque * gear_ratio) / wheel_radius
    """
    return torque * gear_ratio / wheel_radius
 
 
def acceleration(force: float, mass: float = MASS) -> float:
    """
    Calculates the car's acceleration from the force at the wheels using a = F / m.
    Use this function after finding wheel force to get the car's acceleration.
 
    Inputs:
    force: float, force at the wheels (N)
    mass: float, mass of the car with driver (kg)
 
    Outputs:
    float, acceleration (m/s^2)
    """
    return force / mass


def step(state: State) -> State:
    """
    Advances the simulation by one time step.
    Use this function once per frame to get the car's next state.
 
    Inputs:
    state: State, the car's state at the current time step
 
    Outputs:
    State, the car's state at the next time step
    """
    throttle = driver_input(state.t)
    torque   = command_torque(throttle)
    force    = wheel_force(torque)
    a        = acceleration(force)
 
    v = state.v + a * DT   # update velocity first, then use it to update position
    x = state.x + v * DT
 
    return State(t=state.t + DT, x=x, v=v, a=a, throttle=throttle, torque=torque, force=force)
 

state = State()
times = []
accels = []
 
def animate (i):
    global state
    if state.t < END_TIME:   # stop once we reach 22 seconds
        state = step(state)
        times.append(state.t)
        accels.append(state.a)
        line.set_data(times, accels)
    return line,
 
 
fig = plt.figure(figsize=(3,3), dpi=150)
ax = fig.add_subplot(111)
ax.grid()
ax.set_xlim(0, 22)
ax.set_ylim(0, 10)
# these lines are so the animation doesnt zoom in or out
ax.set_xlabel("time (s)")
ax.set_ylabel("acceleration (m/s^2)")
line, = ax.plot([], [])
plt.pause(3)
ani = animation.FuncAnimation(fig, animate, interval=0)
fig.canvas.draw()
plt.show()
 