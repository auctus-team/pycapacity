"""
A performance analysis of the polytope evaluation functions with pinocchio

Running this script will output the mean, variance and max time of the polytope evaluation functions

WARNING it might take some time to run (more than 1 minute - usually less then 5)

Dependencies:
    - pycapacity
    - pinocchio
    - example-robot-data

"""
import pycapacity.robot as capacity # robot capacity module
import numpy as np
import time


import pinocchio as pin
import numpy as np
import time

from example_robot_data import load

# get panda robot using example_robot_data
robot = load('panda')

# get joint position ranges
q_max = robot.model.upperPositionLimit.T
q_min = robot.model.lowerPositionLimit.T
# get max velocity
dq_max = robot.model.velocityLimit
dq_min = -dq_max
# max torque
t_max = robot.model.effortLimit
t_min = -t_max

N1 = 1000 # number of iterations for algorithms force, velocity, acceleration and force sum adn reachable space polytope
N2 = 100 # number of iterations for algorithms force intersection

# Force polytope test
print("Force polytope test")
# execution time and variance
s = []
for i in range(N1):

    q0 = np.random.uniform(q_min,q_max) # random configuration

    # calculate the jacobian
    data = robot.model.createData()
    pin.framesForwardKinematics(robot.model,data,q0)
    pin.computeJointJacobians(robot.model,data, q0)
    J = pin.getFrameJacobian(robot.model, data, robot.model.getFrameId(robot.model.frames[-1].name), pin.LOCAL_WORLD_ALIGNED)
    # use only position jacobian
    J = J[:3,:]
    s1 = time.time()
    f_poly = capacity.force_polytope(J, t_min, t_max) # calculate the polytope vertices and faces
    s.append(time.time()-s1)

# mean and variance and max
print("mean time", np.mean((s)))
print("variance", np.std((s)))
print("max time", np.max((s)))


# Velocity polytope test
print("Velocity polytope test")
# execution time and variance
s = []
for i in range(N1):

    q0 = np.random.uniform(q_min,q_max) # random configuration

    # calculate the jacobian
    data = robot.model.createData()
    pin.framesForwardKinematics(robot.model,data,q0)
    pin.computeJointJacobians(robot.model,data, q0)
    J = pin.getFrameJacobian(robot.model, data, robot.model.getFrameId(robot.model.frames[-1].name), pin.LOCAL_WORLD_ALIGNED)
    # use only position jacobian
    J = J[:3,:]

    s1 = time.time()
    f_poly = capacity.velocity_polytope(J, dq_max, dq_min) # calculate the polytope vertices and faces
    s.append(time.time()-s1)

# mean and variance and max
print("mean time", np.mean((s)))
print("variance", np.std((s)))
print("max time", np.max((s)))

# Acceleration polytope test
print("Acceleration polytope test")
# execution time and variance
s= []
for i in range(N1):

    q0 = np.random.uniform(q_min,q_max) # random configuration

    # calculate the jacobian
    data = robot.model.createData()
    pin.framesForwardKinematics(robot.model,data,q0)
    pin.computeJointJacobians(robot.model,data, q0)
    J = pin.getFrameJacobian(robot.model, data, robot.model.getFrameId(robot.model.frames[-1].name), pin.LOCAL_WORLD_ALIGNED)
    # use only position jacobian
    J = J[:3,:]

    # get mass matrix
    M = pin.crba(robot.model, data, q0)
    
    s1 = time.time()
    f_poly = capacity.acceleration_polytope(J,M, t_min, t_max) # calculate the polytope vertices and faces
    s.append(time.time()-s1)

# mean and variance and max
print("mean time", np.mean((s)))
print("variance", np.std((s)))
print("max time", np.max((s)))


# Force sum polytope test
print("Force sum polytope test")
# execution time and variance
s= []
for i in range(N1):

    q0 = np.random.uniform(q_min,q_max) # random configuration

    # calculate the jacobian
    data = robot.model.createData()
    pin.framesForwardKinematics(robot.model,data,q0)
    pin.computeJointJacobians(robot.model,data, q0)
    J1 = pin.getFrameJacobian(robot.model, data, robot.model.getFrameId(robot.model.frames[-1].name), pin.LOCAL_WORLD_ALIGNED)
    # use only position jacobian
    J1 = J1[:3,:]

    q0 = np.random.uniform(q_min,q_max) # random configuration

    # calculate the jacobian
    data = robot.model.createData()
    pin.framesForwardKinematics(robot.model,data,q0)
    pin.computeJointJacobians(robot.model,data, q0)
    J2 = pin.getFrameJacobian(robot.model, data, robot.model.getFrameId(robot.model.frames[-1].name), pin.LOCAL_WORLD_ALIGNED)
    # use only position jacobian
    J2 = J2[:3,:]

    s1 = time.time()
    f_poly = capacity.force_polytope_sum(J1, J2, t_min, t_max, t_min, t_max) # calculate the polytope vertices and faces
    s.append(time.time()-s1)

# mean and variance and max
print("mean time", np.mean((s)))
print("variance", np.std((s)))
print("max time", np.max((s)))


# Force intersection polytope test
print("Force intersection polytope test")
# execution time and variance
s= []
for i in range(N2):

    q0 = np.random.uniform(q_min,q_max) # random configuration

    # calculate the jacobian
    data = robot.model.createData()
    pin.framesForwardKinematics(robot.model,data,q0)
    pin.computeJointJacobians(robot.model,data, q0)
    J1 = pin.getFrameJacobian(robot.model, data, robot.model.getFrameId(robot.model.frames[-1].name), pin.LOCAL_WORLD_ALIGNED)
    # use only position jacobian
    J1 = J1[:3,:]

    q0 = np.random.uniform(q_min,q_max) # random configuration

    # calculate the jacobian
    data = robot.model.createData()
    pin.framesForwardKinematics(robot.model,data,q0)
    pin.computeJointJacobians(robot.model,data, q0)
    J2 = pin.getFrameJacobian(robot.model, data, robot.model.getFrameId(robot.model.frames[-1].name), pin.LOCAL_WORLD_ALIGNED)
    # use only position jacobian
    J2 = J2[:3,:]

    s1 = time.time()
    f_poly = capacity.force_polytope_intersection(J1, J2, t_min, t_max, t_min, t_max) # calculate the polytope vertices and faces
    s.append(time.time()-s1)

# mean and variance and max
print("mean time", np.mean((s)))
print("variance", np.std((s)))
print("max time", np.max((s)))



# Reaching space polytope test
print("Reaching space polytope test")
# execution time and variance
s= []
for i in range(N1):

    q0 = np.random.uniform(q_min,q_max) # random configuration

    # calculate the jacobian
    data = robot.model.createData()
    pin.framesForwardKinematics(robot.model,data,q0)
    pin.computeJointJacobians(robot.model,data, q0)
    J = pin.getFrameJacobian(robot.model, data, robot.model.getFrameId(robot.model.frames[-1].name), pin.LOCAL_WORLD_ALIGNED)
    # use only position jacobian
    J = J[:3,:]

    # get mass matrix
    M = pin.crba(robot.model, data, q0)

    s1 = time.time()
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
    s.append(time.time()-s1)

# mean and variance and max
print("mean time", np.mean((s)))
print("variance", np.std((s)))
print("max time", np.max((s)))

