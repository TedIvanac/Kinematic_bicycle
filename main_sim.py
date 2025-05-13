import numpy as np
from scipy.integrate import solve_ivp
from bicycle_system import bicycle_system
import matplotlib.pyplot as plt

#start = [5,4]
#goal = [100,70]

#Initial State
x = 0 #starting y coordinate
y = 0 #starting y coordinate
Theta = 0 #Starting orientation of vehicle
vel = 8 #initial velocity
int_e_v = 0
int_e_s = 0
init_state = np.array([x,y,Theta,vel,int_e_v,int_e_s])

#Time parameters
t_end = 60
dt = 0.00001
time = range(0,t_end,dt)

#solve odes using runge-kutta 4,5
[t,output_state] = solve_ivp(bicycle_system,time,init_state)

#Initialize arrays to store results
x_history = output_state[0]
y_history = output_state[1]
theta_history = output_state[2]
vel_history = output_state[3]

plt.plot(x_history,y_history)
plt.show