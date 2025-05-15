import numpy as np
import math

def Kinematics_Description(state):
    ################################################################################################################
    #The kinematic equations for a kinematic bicycle model with realistic force modelling
    ################################################################################################################

    #extract state variables for vehicle from input
    theta = state[0] #yaw of vehicle
    delta = state[1] #steering wheel angle of vehicle
    v = state[2] #velocity of vehicle
    throttle = state[3] #throttle force of vehicle

    # Bicycle Model Parameters
    m = 1519.988   # Mass of the bicycle and rider (kg)
    lf = 0.9       # Distance from the center of mass to the front wheel (m)
    lr = 0.9       # Distance from the center of mass to the rear wheel (m)
    rho = 1.2      # density of air
    Cd = .4        # drag coeff for typical 4 door car
    A = 2.2914     # cross sectional area of a Chevrolet Camaro
    uw = 4.1667    # wind velocity
    g = 9.8        # gravitational constant
    W = m*g        # gravitational force
    f = 0.02       # coeff of rolling resistance
    theta_g = 0    # Angle of vehicle with ground

    Da = .5*rho*Cd*A*(v + uw)^2    # Aerodynamic drag of vehicle

    Xdot = v * math.cos(theta)
    Ydot = v * math.sin(theta)
    Omega = v * (math.tan(delta)/(lf+lr))
    acceleration = (throttle - W*math.sin(theta_g) - f*W*math.cos(theta_g) - Da) / m

    # Create a column vector with derivatives of x, y, theta, delta, and v
    output_state = np.array([Xdot, Ydot,Omega, acceleration])
    return output_state