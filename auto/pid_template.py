import numpy as np


def make_car(desired_v:float=20.0, dt:float=0.1) -> dict:
    """ 
    Generates a dictionary that holds all the car's values. Keeps track of state varaibles.
    """
    car_state_dictionary : dict[str, float] = {
        "v" : 0, #velocity of your car 
        "a" : 0, #acceleration of your car
        "t" : 0, #time of your car
        "x" : 0, #position of your car
        "dt" : dt, #time step of your car, how much the time changes every time you update/step
        "desired_v" : desired_v, #desired velocity of your car, the velocity you want to maintain
        "step" : 0,
    
        #hint: use these variables in the integral and derivative portion of your PID control (steps 5 and 6 )
        "error_prev" : None,
        "net_integral" : 0.0
    }
    return car_state_dictionary

def update(car: dict, throttle_perc: float, mass: float = 1000, max_throttle_force: float = 5000, friction: float = 2.0) -> None:
        """
        Updates the car's state variables based on the throttle percentage.
        Use this function after finding throttle percentage to update the car's state variables.

        Inputs:
        car: dictionary containing the car's state variables
        throttle_perc: float, throttle percentage (-1 to 1)

        Outputs:
        None, but updates the car's state variables
        """
        force = throttle_perc * max_throttle_force
        car["a"] = (force / mass) - friction
        car["v"] += car["a"] * car["dt"]
        car["x"] += car["v"] * car["dt"]
        car["t"] += car["dt"]
        car["step"] += 1


def calculate_desired_acceleration(car: dict, K_P: float, K_I: float = 0.0, K_D: float = 0.0) -> tuple[float, float]:
        """
        Calculates the velocity error and desired acceleration of the car to
        bring the car's velocity toward the desired velocity.
        desired acceleration = K_P * error + K_I * net_integral + K_D * (de / dt).
        Includes anti-windup (conditional integration): the integral only accumulates when doing so
        won't ask for more than the motor's max acceleration, which prevents overshoot from
        the integral building up while the throttle is fully saturated.
        
        Inputs:
        car: dictionary containing the car's state variables 
             (uses car["v"], car["desired_v"], car["dt"], car["net_integral"], car["error_prev"])
        K_P: float, proportional gain, scales how strongly the controller reacts to the current error
        K_I: float, integral gain, scales how strongly the controller reacts to the accumulated past error
        K_D: float, derivative gain, scales how strongly the controller reacts to the rate of change of the error

        Outputs:
        tuple (acceleration_desired, error):
                acceleration_desired: float, desired acceleration in m/s^2 (positive = speed up, negative = slow down)
                error: float, desired velocity minus current velocity in m/s
        """

        error = car["desired_v"] - car["v"] # desired - actual: positive error = speed up

        if car["error_prev"] is None: #first iteration has no prev error, so skip derivative
                derivative = 0.0
        else:
                derivative = (error - car["error_prev"]) / car["dt"] # de/dt
        car["error_prev"] = error # save for next iteration

        # calculate the PD terms (don't depend on integral decision)
        pd_terms = K_P * error + K_D * derivative

        
        # Integral with anti-windup (conditional integration): only accept this step's integral if the
        # result wouldn't ask for more than the motor's max acceleration (5000 N / 1000 kg)
        trial_integral = car["net_integral"] + error * car["dt"]
        if abs(pd_terms + K_I * trial_integral) < 5.0:
                car["net_integral"] = trial_integral

        # incorporate integral decision into full PID command
        acceleration_desired = pd_terms + K_I * car["net_integral"]
        return acceleration_desired, error



def acceleration_to_throttle_percentage(acceleration_desired: float, mass: float = 1000, max_throttle_force: float = 5000) -> float:
        """
        Converts a desired acceleration into a throttle percentage for the motor.

        Inputs:
        acceleration_desired: float, desired acceleration in m/s^2 (from calculate_desired_acceleration)
        mass: float, mass of the car in kg
        max_throttle_force: float, maximum force the motor can produce in N

        Outputs:
        throttle_perc: float, fraction of max_throttle_force to apply, from range -100% to 100%

        Raises:
        ValueError: if mass or max_throttle_force is not positive
        """

        # Check for valid inputs
        if mass <= 0 or max_throttle_force <= 0:
                raise ValueError("mass and max_throttle_force must be positive")

        max_acceleration = max_throttle_force / mass  # Newton's 2nd law: a = F/m
        throttle__perc = acceleration_desired / max_acceleration
        return float(np.clip(throttle__perc, -1.0, 1.0))  # set throttle_perc range from -100% to 100%