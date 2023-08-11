"""
A performance analysis of the polytope evaluation functions with biorbd

Running this script will output the mean, variance and max time of the polytope evaluation functions

WARNING it might take some time to run (more than 1 minute - usually less then 5)

Dependencies:
    - biorbd
    - pycapacity


"""

import numpy as np
import biorbd

import pycapacity.human as capacity # robot capacity module
import numpy as np
import time


# Load a predefined model
model = biorbd.Model("/home/antun/gitlab/biomechanics/pyomeca_models/MOBL_ARMS_fixed_33.bioMod")

# get the number of dof and muslces
nq = model.nbQ()
nb_mus = model.nbMuscles()

# getting joint min and max
q_min = []
q_max= []
for s in range(model.nbSegment()):
    for i in model.segment(s).QRanges():
        q_min.append(i.min())
        q_max.append(i.max())
q_min = np.array(q_min)
q_max = np.array(q_max)

# contraction velocity limits max and min
dl_max = np.ones(nb_mus)*10  
dl_min = -np.ones(nb_mus)*10

N_iter = 50 # number of iterations per algorithm 

# Force polytope test
print("Force polytope test")
# execution time and variance
s= []
for i in range(N_iter):

    q = np.random.uniform(q_min, q_max)

    model.updateMuscles(q, True)
    model.UpdateKinematicsCustom(q, np.zeros(nq), np.zeros(nq))

    F_max = []
    F_min = []
    for i in range(nb_mus):
        F_max.append(model.muscle(i).characteristics().forceIsoMax())
        #F_min.append(0)
        a = biorbd.HillThelenType(model.muscle(i)).FlPE()
        if a > 1:
            a = 0.1
        elif a < 0:
            a = 0
        F_min.append(a*F_max[-1])

    N = -model.musclesLengthJacobian(q).to_array().T
    J = model.markersJacobian(q, False, False)[-1].to_array()
    # Proceed with the inverse dynamics
    Tau_grav = model.InverseDynamics(q, np.zeros(nq), np.zeros(nq))
    
    start = time.time()
    f_poly = capacity.force_polytope(J, N, F_min, F_max, 10, -Tau_grav.to_array())
    s.append(time.time() - start)

# mean and variance and max
print("mean time", np.mean((s)))
print("variance", np.std((s)))
print("max time", np.max((s)))


# Velocity polytope test
print("Velocity polytope test")
# execution time and variance
s = []
for i in range(N_iter):

    q = np.random.uniform(q_min, q_max)

    model.updateMuscles(q, True)
    model.UpdateKinematicsCustom(q, np.zeros(nq), np.zeros(nq))

    N = -model.musclesLengthJacobian(q).to_array().T
    J = model.markersJacobian(q, False, False)[-1].to_array()
    
    start = time.time()
    f_poly = capacity.velocity_polytope(J=J, N=N, dl_min=dl_min, dl_max=dl_max, tol=10)
    s.append(time.time() - start)

# mean and variance and max
print("mean time", np.mean((s)))
print("variance", np.std((s)))
print("max time", np.max((s)))


# Acceleration polytope test
print("Acceleration polytope test")
# execution time and variance
s= []
for i in range(N_iter):

    q = np.random.uniform(q_min, q_max)

    model.updateMuscles(q, True)
    model.UpdateKinematicsCustom(q, np.zeros(nq), np.zeros(nq))

    F_max = []
    F_min = []
    for i in range(nb_mus):
        F_max.append(model.muscle(i).characteristics().forceIsoMax())
        #F_min.append(0)
        a = biorbd.HillThelenType(model.muscle(i)).FlPE()
        if a > 1:
            a = 0.1
        elif a < 0:
            a = 0
        F_min.append(a*F_max[-1])

    N = -model.musclesLengthJacobian(q).to_array().T
    J = model.markersJacobian(q, False, False)[-1].to_array()
    M = model.massMatrix(q).to_array()

    start = time.time()
    f_poly = capacity.acceleration_polytope(J=J,N=N,M=np.array(M), F_max=F_max, F_min=F_min, tol=10) # calculate the polytope vertices and faces
    s.append(time.time()-start)

# mean and variance and max
print("mean time", np.mean((s)))
print("variance", np.std((s)))
print("max time", np.max((s)))


