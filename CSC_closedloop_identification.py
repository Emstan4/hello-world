# -*- coding: utf-8 -*-
"""
Created on Wed Feb 17 12:58:42 2016

@author: Charles
"""

from __future__ import division
import numpy as np
from control import tf, ss
from numpy import linalg
import scipy

# Exact Process Model
K = 1
tau = 5
numer = [K]
denom = [tau,1]
sys1 = tf(numer,denom)

Kc = 2
tau_i = 5


#PI controller
controllerA = [Kc*tau_i, Kc]
controllerB = [tau_i, 0]

cont_sys1 = tf(controllerA, controllerB)
cont_sys = ss(cont_sys1)
#Transformation to State-Space

sys = ss(sys1)
A, B, C, D = np.asarray(sys.A), np.asarray(sys.B), np.asarray(sys.C), \
    np.asarray(sys.D)

contA, contB, contC, contD = np.asarray(cont_sys.A), np.asarray(cont_sys.B), np.asarray(cont_sys.C), \
    np.asarray(cont_sys.D)
    
Nstates = A.shape[0]
contNstates = contA.shape[0]

z_cont = np.zeros((contNstates, 1))
z = np.zeros((Nstates, 1))

t_start = 0
t_end = 50
dt = 0.1
tspan = np.arange(t_start, t_end, dt)
npoints = len(tspan)

#List for storages
yplot = []
phi_T = []
#sigma
deviation = 1.0
#Initial conditions
z = 0
y = 0

#Sampling parameters

delta = 1# sampling interval
next_time = 0
next_time2 = delta

y_sp = 1

a = np.exp(-delta/tau)
b = K*(1 - a)
a = np.exp(-delta/tau)
b = K*(1-a)
y_1 = 0
u = u_1 = 0
j = 0

my_sum = np.zeros((2,2))
my_sum2 = np.zeros((2,1))
y_list = []

deviation = 0.5
next_time3 = 0
next_time4 = 2*delta

period = (1/2*np.pi)*0.2

for i, t in enumerate(tspan):
    noise = deviation*np.random.rand()
    signal = scipy.signal.square(period*t, duty = 0.5)
    
    if t >= next_time:
        if t >= next_time2:
            phi_T.append([y_1, u_1])

            phi = np.matrix.transpose(np.array(phi_T))
            y_list.append([y])
            product = np.dot(phi, phi_T)
            product2 = np.dot(phi, y_list)
            my_sum += product
            my_sum2 += product2
            
            next_time2 += delta
    

        y_1 = y
        u_1 = u
        phi_T = []
        y_list = []
        
        
        next_time += delta
    
    yplot.append(y)
    e = y_sp - y
    
    dzcdt = contA*z_cont + contB*e
    u = contC*z_cont + contD*e
    u = u[0,0]
        
    dzdt = A*z + B*u
    y = C*z + D*u
    y = y[0,0]
    y += noise
    
    z += dzdt*dt
    z_cont += dzcdt*dt

r = linalg.inv(my_sum)
parameters = np.dot(r, my_sum2)        
r = linalg.inv(my_sum)
parameters = np.dot(r, my_sum2)   
print ('a_esti =',parameters[0,0], 'b_esti = ', parameters[1,0] )
print ('a_real =',a, 'b_real = ', b)
