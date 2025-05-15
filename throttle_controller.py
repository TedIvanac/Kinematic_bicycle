import numpy as np

def throttle_controller(target_vel, cur_vel, error_integral):
    ################################################################################################################
    #PID Controller to control throttle force for the vehicle
    ################################################################################################################
    
    #if not throttle_controller.errorlist:
       # throttle_controller.errorlist = 0
    
    #error = target velocity - current velocity
    error = target_vel - cur_vel

    #Gains - Untuned
    Pgain = 1
    Igain = 0
    Dgain = 0

    #Derivative
    #errorlist(len(errorlist)+1) = error
    throttle_controller.errorlist = np.append(throttle_controller.errorlist,error)
    der_error = throttle_controller.errorlist[-1] - throttle_controller.errorlist[-2]

    # PID
    P = Pgain*error
    I = Igain*error_integral
    D = Dgain*der_error

    # PID = P+I+D
    #Currently just P controller
    
    vel_ctrl = [P, error]
    return np.array(vel_ctrl)

throttle_controller.errorlist = np.array([])