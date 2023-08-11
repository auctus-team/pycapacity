"""
A performance analysis of the polytope evaluation functions

Running this script will output the mean, variance and max time of the polytope evaluation functions

WARNING it might take some time to run (more than 1 minute - usually less then 5)
"""
import pycapacity.robot as capacity # robot capacity module
import numpy as np
import time



m = 3 # 3d forces
n = 7 # robot dof

N1 = 1000 # number of iterations for algorithms force, velocity, acceleration and force sum polytope
N2 = 100 # number of iterations for algorithms force intersection and reachable space

# Force polytope test
print("Force polytope test")
# execution time and variance
s= [time.time()]
for i in range(N1):
    J = np.array(np.random.rand(m,n))*2-1 # random jacobian matrix

    t_max = np.ones(n)  # joint torque limits max and min
    t_min = -np.ones(n)

    f_poly = capacity.force_polytope(J, t_min, t_max) # calculate the polytope vertices and faces
    s.append(time.time())

# mean and variance and max
print("mean time", np.mean(np.diff(s)))
print("variance", np.std(np.diff(s)))
print("max time", np.max(np.diff(s)))


# Velocity polytope test
print("Velocity polytope test")
# execution time and variance
s= [time.time()]
for i in range(N1):
    J = np.array(np.random.rand(m,n))*2-1 # random jacobian matrix

    dq_max = np.ones(n)  # joint torque limits max and min
    dq_min = -np.ones(n)

    f_poly = capacity.velocity_polytope(J, dq_max, dq_min) # calculate the polytope vertices and faces
    s.append(time.time())

# mean and variance and max
print("mean time", np.mean(np.diff(s)))
print("variance", np.std(np.diff(s)))
print("max time", np.max(np.diff(s)))

# Acceleration polytope test
print("Acceleration polytope test")
# execution time and variance
s= [time.time()]
for i in range(N1):
    J = np.array(np.random.rand(m,n))*2-1 # random jacobian matrix
    M = np.array(np.random.rand(n,n)) # random jacobian matrix

    t_max = np.ones(n)  # joint torque limits max and min
    t_min = -np.ones(n)

    f_poly = capacity.acceleration_polytope(J,M, t_min, t_max) # calculate the polytope vertices and faces
    s.append(time.time())

# mean and variance and max
print("mean time", np.mean(np.diff(s)))
print("variance", np.std(np.diff(s)))
print("max time", np.max(np.diff(s)))



# Force sum polytope test
print("Force sum polytope test")
# execution time and variance
s= [time.time()]
for i in range(N1):
    J1 = np.array(np.random.rand(m,n))*2-1 # random jacobian matrix
    J2 = np.array(np.random.rand(m,n))*2-1 # random jacobian matrix

    t1_max = np.ones(n)  # joint torque limits max and min
    t1_min = -np.ones(n)

    t2_max = np.ones(n)  # joint torque limits max and min
    t2_min = -np.ones(n)

    f_poly = capacity.force_polytope_sum(J1, J2, t1_min, t1_max, t2_min, t2_max) # calculate the polytope vertices and faces
    s.append(time.time())

# mean and variance and max
print("mean time", np.mean(np.diff(s)))
print("variance", np.std(np.diff(s)))
print("max time", np.max(np.diff(s)))


# Force intersection polytope test
print("Force intersection polytope test")
# execution time and variance
s= [time.time()]
for i in range(N2):
    J1 = np.array(np.random.rand(m,n))*2-1 # random jacobian matrix
    J2 = np.array(np.random.rand(m,n))*2-1 # random jacobian matrix

    t1_max = np.ones(n)  # joint torque limits max and min
    t1_min = -np.ones(n)

    t2_max = np.ones(n)  # joint torque limits max and min
    t2_min = -np.ones(n)

    f_poly = capacity.force_polytope_intersection(J1, J2, t1_min, t1_max, t2_min, t2_max) # calculate the polytope vertices and faces
    s.append(time.time())

# mean and variance and max
print("mean time", np.mean(np.diff(s)))
print("variance", np.std(np.diff(s)))
print("max time", np.max(np.diff(s)))



# Reaching space polytope test
print("Reaching space polytope test")
# execution time and variance
s= [time.time()]
for i in range(N2):
    J = np.array(np.random.rand(m,n))*2-1 # random jacobian matrix
    M = np.array(np.random.rand(n,n)) # random jacobian matrix

    t_max = np.ones(n)  # joint torque limits max and min
    t_min = -np.ones(n)

    q_max = np.ones(n)  # joint position limits max and min
    q_min = -np.ones(n)

    dq_max = np.ones(n)  # joint velocity limits max and min
    dq_min = -np.ones(n)

    # initial configuration
    q0 = np.array(np.random.rand(n))*2-1
    # calculate the polytope vertices and faces 
    f_poly = capacity.reachable_space_approximation( M = M,
                                                    J= J,
                                                    q0 = q0,
                                                    t_min= t_min,
                                                    t_max= t_max,
                                                    q_min=q_min,
                                                    q_max=q_max, 
                                                    dq_min=dq_min, 
                                                    dq_max=dq_max,
                                                    horizon=0.3) 
    s.append(time.time())

# mean and variance and max
print("mean time", np.mean(np.diff(s)))
print("variance", np.std(np.diff(s)))
print("max time", np.max(np.diff(s)))

