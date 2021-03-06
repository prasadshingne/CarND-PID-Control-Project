#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  2 16:44:05 2021

@author: sprasad
"""
import subprocess
import os
import pandas as pd
from scipy.optimize import minimize
import numpy as np

#%% Normalize and denormalize Kp, Ki, Kd for the optimizer
def denormalize(v,x0,x1):
    v_denorm = []
    ii = 0
    while ii < len(v):
        v_denorm.append((v[ii]+1)*(x1[ii]-x0[ii])/2)
        ii += 1
    return v_denorm

def normalize(v,x0,x1):
    v_norm = []
    ii = 0
    while ii < len(v):
        v_norm.append(-1+ (2/(x1[ii]-x0[ii]) * v[ii]))
        ii += 1
    return v_norm
    
#%% Function takes p = normalized[Kp, Ki, Kd], runs the simulion and return cross track error
def run_pid(p):
    
    home = os.getcwd() # store current working directory
    os.chdir('../build') # change to build directory where the executable is
    
    # check if there exists an old init file
    if os.path.isfile('pid.init'):
        os.remove('pid.init')
        
    p_init = []
    p_init = denormalize(p,xlow,xhigh)
    
    # write a new init file
    p_init = pd.DataFrame(p_init)
    p_init = p_init.transpose()
    p_init.to_csv('pid.init', header=None, index=None, sep=' ')
        
    # execute pid
    subprocess.run('./pid') # run executable
    
    # read simulation result
    sim_res = pd.read_csv('output.txt', sep=' ', names=['tstep','cross_track_error',
                                                         'steering_ang','speed_error','throttle','speed'])
    
    # calculate RMS error and normalize for the cross track and speed
    rmse_steer = ((sim_res.cross_track_error ** 2).mean() ** 0.5)

    ## Code below was not used --> tried weighted sum of CTE and speed error, also tried using cumulative sum
    ## instead of root mean squared error
    
    ## steer_err  = np.cumsum(sim_res.cross_track_error)
    ## rmse_speed = ((sim_res.speed_error ** 2).mean() ** 0.5) / 45
    ## steer_err  = ((sim_res.speed_error ** 2).mean() ** 0.5)

    ## calculate the weighted sum of the RMS errors
    ## rmse_weighted = 2/3*rmse_steer + 1/3*rmse_speed
    ## error = np.array(steer_err)

    
    error = rmse_steer
    
    # change back to home directory
    os.chdir(home)  
    
    # return the error to minimize
    return error

#%%    
# initial guess of PID
p1 = [0.15, 0.0002, 3.0] # --> selected [0.14937642030249174, 0.0008028211368793284, 3.355222551300104] 


# bounds Kp    Ki   Kd
xlow  = [0.05, 0.0,     0.1]
xhigh = [0.25, 0.0025 , 6.0]

# normalize bounds to [-1, 1]
xinit = normalize(p1,xlow,xhigh)

print(xinit)

bnds  = ((-1, 1),(-1, 1),(-1, 1))

# res = run_pid(xinit)

# Use minimize function from scipy. Sequential Least Square Programming method is used with the above
# initial condition and bounds. Optimization is continuesd till result does not change more than tol
# Maximum normalized step size is set to 0.05 and maximum iterations allowed is 200
res = minimize(run_pid, xinit, method='SLSQP', bounds = bnds, tol = 1e-6, options={'eps':0.05,'disp':True, 'maxiter':200})

xout = denormalize(res.x, xlow, xhigh)
print(xout)