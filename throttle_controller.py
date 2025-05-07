import numpy as np

def throttle_controller(target_vel, cur_vel, error_integral):
    if not errorlist:
        errorlist = 0
    
    #error = target velocity - current velocity
    error = target_vel - cur_vel

    #Gains - Untuned
    Pgain = 175
    Igain = 37
    Dgain = .4

    #Derivatives
    errorlist(len(errorlist)+1) = error
    der_error = errorlist(len(errorlist)-1) - errorlist(len(errorlist-1)-1)

    # PID
    P = Pgain*error
    I = Igain*error_integral
    D = Dgain*der_error

    # PID = P+I+D
    #Currently just P controller
    
    vel_ctrl = [P, error]
    return np.array(vel_ctrl)