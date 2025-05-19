import numpy as np
import math
from test_path_1 import waypointX
from test_path_1 import waypointY
from throttle_controller import throttle_controller
from steering_controller import steering_controller
from kinematics_description import Kinematics_Description
from pidcontroller import PIDController

def bicycle_system(time, init_state):
    
    #Initial State
    x_coord = init_state[0] #starting y coordinate
    y_coord = init_state[1] #starting y coordinate
    theta = init_state[2] #Starting orientation of vehicle
    vel = init_state[3] #initial velocity

    #speed setpoint for the vehicle
    speed = 5

    #Stop car if all waypoints have been reached
    if np.any(waypointX != float('inf')):
        waypoint_counter = 0
        while(waypoint_counter < len(waypointX)):
            #calculate difference arrays between waypoints and current coordinates
            xdiff = waypointX[waypoint_counter] - x_coord
            ydiff = waypointY[waypoint_counter] - y_coord
            #use atan2 to find the angle vehicle must be at to reach next point
            angle = np.atan2(xdiff,ydiff)

            #giving some tolerance for reaching the waypoint
            diff = np.sqrt((xdiff**2)+(ydiff**2))
            if(diff <= .5):
                print("Waypoint Reached!")
                waypoint_counter += 1
            
            vel_ctrl = PIDController(Kp=15.0,Ki=0.1,Kd=0,setpoint=speed)
            steer_ctrl = PIDController(Kp=1.0,Ki=0,Kd=0,setpoint=angle)

            throttle = vel_ctrl.compute(vel,time)
            delta = steer_ctrl.compute(theta,time)
            if delta > 0.7854:
                delta = 0.7854
            elif delta < -0.7854:
                delta = -0.7854
            out = Kinematics_Description(np.array([theta,delta,vel,throttle]))
    else:
        print("All Waypoints Reached!")
        #err_throttle = 0
        #err_theta = 0
        out = np.array([0,0,0,.02])
    
    #dydt = np.array([*out,err_throttle,err_theta])
    dydt = np.array([*out])
    return dydt

