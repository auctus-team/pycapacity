import pycapacity.robot as robot
import numpy as np

def test1():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    M = np.random.rand(n,n)
    t_min = np.zeros(n)
    t_max = np.ones(n)
    dq_min = -t_max
    dq_max = t_max

    robot.force_polytope(J, t_min, t_max)
    robot.force_polytope_withfaces(J, t_min, t_max)
    robot.acceleration_polytope(J, M, t_min, t_max)
    robot.acceleration_polytope_withfaces(J, M, t_min, t_max)
    robot.velocity_polytope(J, dq_min, dq_max)
    robot.velocity_polytope_withfaces(J, dq_min, dq_max)

