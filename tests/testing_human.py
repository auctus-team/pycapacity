import pycapacity.human as human
import numpy as np

def test1():
    L = 20 # nb muslces
    n = 5 # nb joints
    m = 3 # cartesian forces
    N = (np.random.rand(n,L)*2 -1)
    J = np.random.rand(m,n)
    M = np.random.rand(n,n)
    F_min = np.zeros(L)
    F_max = np.ones(L)

    human.force_polytope(J,N, F_min, F_max, 0.1)
    human.acceleration_polytope(J, N, M, F_min, F_max)
    # direct acceleration polytope calculation
    human.iterative_convex_hull_method(np.identity(m), J.dot(np.linalg.inv(M).dot(N)), F_min, F_max,0.1)
    human.acceleration_polytope(J, N, M, F_min, F_max)

