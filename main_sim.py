import numpy as np
import scipy
from bicycle_system import bicycle_system

start = [5,4]
goal = [100,70]

#Initial State
X = start(0) #starting y coordinate
Y = start(1) #starting y coordinate
V = 8 #initial velocity
int_e_v = 0
int_e_s = 0
init_state = np.array([X,Y,V,int_e_v,int_e_s])

#Time parameters
t_end = 60
dt = 0.00001
time = range(0,t_end,dt)

#solve odes using runge-kutta 4,5
[t,y] = scipy.integrate.solve_ivp(bicycle_system,time,init_state)