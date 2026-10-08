# PID Car Controller

A PID controller that makes a simple 1D car reach and hold a target velocity.

## How it works

Each time step, the controller compares the car's velocity to the target velocity to get the error.
It uses the PID formula (see pid-equation-all-terms.png) to turn that error into a desired acceleration, then converts that into a throttle percentage between -100% and 100%.
The car's velocity and position are then updated based on the throttle and friction.

- P (proportional) reacts to the current error
- I (integral) reacts to error built up over time, which cancels out friction
- D (derivative) reacts to how fast the error is changing, which reduces overshoot

## Files

- pid_template.py: the car model and the PID controller
- run_template.py: runs the simulation and plots velocity, error and friction over time

## How to run

Install the libraries:
    
    Mac:     python3 -m pip install numpy matplotlib
    Windows: py -m pip install numpy matplotlib

Run the simulation:

    Mac:     python3 run_template.py
    Windows: py run_template.py

The gains (K_P, K_I, K_D), run time (STEPS and DT), target velocity (DESIRED_V), and friction changes (FRICTION_SCHEDULE) can all be edited at the top of run_template.py.

## Extensions

Anti-windup: the integral term stops building up while the throttle is at 100%. This removes the overshoot the car had when speeding up to the target.

Variable friction: friction changes at set times during the run. The controller quickly adjusts and brings the velocity back to the target.
