"""
A performance analysis of the polytope evaluation functions

Running this script will output the mean, variance and max time of the polytope evaluation functions

WARNING it might take some time to run (more than 1 minute - usually less then 5)
"""
import pycapacity.human as capacity # robot capacity module
import numpy as np
import time


m = 3 # 3d forces
n = 7 # robot dof
d = 50 # number muscles

N_iter = 50 # number of iterations per algorithm 

# Force polytope test
print("Force polytope test")
# execution time and variance
s= [time.time()]
for i in range(N_iter):
    J = np.array(np.random.rand(m,n))*2-1 # random jacobian matrix
    N = np.array(np.random.rand(n,d))*2-1 # random jacobian matrix

    F_max = np.ones(d)  # joint torque limits max and min
    F_min = -np.ones(d)

    f_poly = capacity.force_polytope(J=J,N=N, F_max=F_max, F_min=F_min, tol=10) # calculate the polytope vertices and faces
    s.append(time.time())

# mean and variance and max
print("mean time", np.mean(np.diff(s)))
print("variance", np.std(np.diff(s)))
print("max time", np.max(np.diff(s)))

# Acceleration polytope test
print("Velocity polytope test")
# execution time and variance
s= [time.time()]
for i in range(N_iter):
    J = np.array(np.random.rand(m,n))*2-1 # random jacobian matrix
    N = np.array(np.random.rand(n,d))*2-1 # random jacobian matrix

    dl_max = np.ones(d)  # joint torque limits max and min
    dl_min = -np.ones(d)

    f_poly = capacity.velocity_polytope(J=J,N=N, dl_max=dl_max, dl_min=dl_min, tol=10) # calculate the polytope vertices and faces
    s.append(time.time())

# mean and variance and max
print("mean time", np.mean(np.diff(s)))
print("variance", np.std(np.diff(s)))
print("max time", np.max(np.diff(s)))



# Acceleration polytope test
print("Acceleration polytope test")
# execution time and variance
s= [time.time()]
for i in range(N_iter):
    J = np.array(np.random.rand(m,n))*2-1 # random jacobian matrix
    M = np.array(np.random.rand(n,n)) # random jacobian matrix
    N = np.array(np.random.rand(n,d))*2-1 # random jacobian matrix

    F_max = np.ones(d)  # joint torque limits max and min
    F_min = -np.ones(d)

    f_poly = capacity.acceleration_polytope(J=J,N=N,M=M, F_max=F_max, F_min=F_min, tol=10) # calculate the polytope vertices and faces
    s.append(time.time())

# mean and variance and max
print("mean time", np.mean(np.diff(s)))
print("variance", np.std(np.diff(s)))
print("max time", np.max(np.diff(s)))


