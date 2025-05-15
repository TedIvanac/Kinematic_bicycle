import numpy as np

def steering_controller(target_angle, cur_angle, error_integral):
    ################################################################################################################
    #PID Controller to control steering for the vehicle
    ################################################################################################################
    if not steering_controller.errorlist:
        steering_controller.errorlist = 0
    
    #error = target steering angle - current steering angle
    error = target_angle - cur_angle

    #Gains - Untuned
    Pgain = 1
    Igain = 0
    Dgain = 0

    #Derivatives
    steering_controller.errorlist += error
    der_error = steering_controller.errorlist(len(steering_controller.errorlist)-1) - steering_controller.errorlist(len(steering_controller.errorlist-1)-1)

    # PID
    P = Pgain*error
    I = Igain*error_integral
    D = Dgain*der_error

    # PID = P+I+D
    PI = P+I+D
    steer_ctrl = [PI, error]
    return np.array(steer_ctrl)

steering_controller.errorlist = 0