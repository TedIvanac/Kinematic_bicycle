import numpy as np

def steering_controller(target_angle, cur_angle, error_integral):
    if not errorlist:
        errorlist = 0
    
    #error = target steering angle - current steering angle
    error = target_angle - cur_angle

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
    PI = P+I+D
    steer_ctrl = [PI, error]
    return np.array(steer_ctrl)