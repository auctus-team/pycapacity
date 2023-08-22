import pycapacity.human as human
import numpy as np


# test all the algos with random data at once
def test_all_at_once_random():
    L = 20 # nb muscles
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

# unit test for joint_torque_polytope function with random data
def test_joint_torques_random():
    L = 20 # nb muscles
    n = 3 # nb joints
    N = (np.random.rand(n,L)*2 -1)
    F_min = np.zeros(L)
    F_max = np.ones(L)

    human.joint_torques_polytope(N, F_min, F_max)
    assert True

# unit test velocity_polytope function with random data
def test_human_velocity_random():
    L = 20 # nb muscles
    n = 5 # nb joints
    m = 3
    N = (np.random.rand(n,L)*2 -1)
    J = np.random.rand(m,n)
    dl_min = np.zeros(L)
    dl_max = np.ones(L)

    human.velocity_polytope(J, N, dl_min , dl_max, tol=1e-5)
    assert True


# unit test for torque_to_muscle_force function with random data
def test_human_torque_to_force_lp():
    L = 20 # nb muscles
    n = 5 # nb joints
    N = (np.random.rand(n,L)*2 -1)
    tau = np.random.rand(n)
    F_min = np.zeros(L)
    F_max = np.ones(L)

    human.torque_to_muscle_force(N, F_min, F_max, tau, options='lp')
    assert True

# unit test for torque_to_muscle_force function with random data
def test_human_torque_to_force_qp():
    L = 20 # nb muscles
    n = 5 # nb joints
    N = (np.random.rand(n,L)*2 -1)
    tau = np.random.rand(n)
    F_min = np.zeros(L)
    F_max = np.ones(L)

    human.torque_to_muscle_force(N, F_min, F_max, tau, options='qp')
    assert True



# unit test for velocity polytope with both elongation and joint velocity max provided
def test_human_force_random():
    L = 20 # nb muscles
    n = 5 # nb joints
    m = 3 # cartesian forces
    N = (np.random.rand(n,L)*2 -1)
    J = np.random.rand(m,n)*2-1
    dl_min = np.zeros(L)
    dl_max = np.ones(L)
    dtheta_min = -np.ones(n)
    dtheta_max = np.ones(n)

    human.velocity_polytope(J, N=N, dl_min=dl_min , dl_max=dl_max, dq_max=dtheta_max, dq_min=dtheta_min, tol=1e-5)
    assert True

# test all the ellipsoid algos with random data at once
def test_all_ellipsoid_at_once_random():
    L = 20 # nb muscles
    n = 5 # nb joints
    m = 3 # cartesian forces
    N = (np.random.rand(n,L)*2 -1)
    J = np.random.rand(m,n)
    M = np.random.rand(n,n)
    F_max = np.ones(L)
    dl_max = np.ones(L)

    human.force_ellipsoid(J,N, F_max)
    human.acceleration_ellipsoid(J, M, N, F_max)
    human.velocity_ellipsoid(J, N, dl_max)