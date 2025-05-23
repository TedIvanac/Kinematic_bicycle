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
vel = 0 #initial velocity
#int_e_v = 0
#int_e_s = 0
#init_state = np.array([x,y,Theta,vel,int_e_v,int_e_s])
init_state = np.array([x,y,Theta,vel])

#Time parameters
t_end = 60
dt = 0.00001
time = np.arange(0,t_end,dt)

output_state = np.empty([6,1])

#solve odes using runge-kutta 4,5
output_state = solve_ivp(bicycle_system,[dt, t_end], init_state)

#Initialize arrays to store results
t = output_state.t
x_history = output_state.y[0]
y_history = output_state.y[1]
theta_history = output_state.y[2]
vel_history = output_state.y[3]

plt.plot(x_history,y_history)
plt.show