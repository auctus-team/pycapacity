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

# unit test for joint_torque_polytope fucntion with randome data
def test2():
    L = 20 # nb muslces
    n = 3 # nb joints
    N = (np.random.rand(n,L)*2 -1)
    F_min = np.zeros(L)
    F_max = np.ones(L)

    human.joint_torques_polytope(N, F_min, F_max)
    assert True

# unit test velocity_polytope fucntion with randome data
def test3():
    L = 20 # nb muslces
    n = 5 # nb joints
    m = 3
    N = (np.random.rand(n,L)*2 -1)
    J = np.random.rand(m,n)
    dl_min = np.zeros(L)
    dl_max = np.ones(L)

    human.velocity_polytope(J, N, dl_min , dl_max, tol=1e-5)
    assert True


# unit test for torque_to_muscle_force fucntion with randome data
def test4():
    L = 20 # nb muslces
    n = 5 # nb joints
    N = (np.random.rand(n,L)*2 -1)
    tau = np.random.rand(n)
    F_min = np.zeros(L)
    F_max = np.ones(L)

    human.torque_to_muscle_force(N, F_min, F_max, tau, options='lp')
    assert True

# unit test for torque_to_muscle_force fucntion with randome data
def test5():
    L = 20 # nb muslces
    n = 5 # nb joints
    N = (np.random.rand(n,L)*2 -1)
    tau = np.random.rand(n)
    F_min = np.zeros(L)
    F_max = np.ones(L)

    human.torque_to_muscle_force(N, F_min, F_max, tau, options='qp')
    assert True