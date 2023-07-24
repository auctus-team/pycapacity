import pycapacity.robot as robot
import numpy as np

# all functions at once unit test with random data
def test_all_at_once_random():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    M = np.random.rand(n,n)
    t_min = np.zeros(n)
    t_max = np.ones(n)
    dq_min = -t_max
    dq_max = t_max

    robot.force_polytope(J, t_min, t_max)
    robot.acceleration_polytope(J, M, t_min, t_max)
    robot.velocity_polytope(J, dq_min, dq_max)


# unit test for force_polytope function with random data   
def test_force_polytope_random():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    t_min = np.zeros(n)
    t_max = np.ones(n)
    robot.force_polytope(J, t_min, t_max)
    assert True

# unit test for velocity_polytope function with random data   
def test_velocity_polytope_random():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    dq_max = np.ones(n)
    robot.velocity_ellipsoid(J,  dq_max)
    assert True

# unit test for force_ellipsoid function with random data   
def test_force_ellipsoid_random():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    t_max = np.ones(n)
    robot.force_ellipsoid(J, t_max)
    assert True

# unit test for acceleration_ellipsoid function with random data   
def test_acceleration_ellipsoid_random():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    M = np.random.rand(n,n)
    t_max = np.ones(n)
    robot.acceleration_ellipsoid(J, M, t_max)
    assert True

# unit test for force_polytope_intersection  function with random data   
def test_force_polytope_intersection_random():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    J2 = np.random.rand(m,n)*2 - 1
    t_min = -np.ones(n)
    t_max = np.ones(n)
    robot.force_polytope_intersection(J, J2, t_max, t_min, t_max, t_min)
    assert True

# unit test for force_polytope_intersection  function with random data    with t_bias
def test_force_polytope_intersection_random_bias():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    J2 = np.random.rand(m,n)*2 - 1
    t_min = -np.ones(n)
    t_max = np.ones(n)
    t_bias = np.zeros(n)
    robot.force_polytope_intersection(J, J2, t_max, t_min, t_max, t_min, t_bias, t_bias)
    assert True

# unit test for force_polytope_sum  function with random data   
def test_force_polytope_sum_random():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    J2 = np.random.rand(m,n)*2 - 1
    t_min = -np.ones(n)
    t_max = np.ones(n)
    robot.force_polytope_sum(J, J2, t_max, t_min, t_max, t_min)
    assert True

# unit test for force_polytope  function with random data 1d matrix J 
def test_force_polytope_random_1d():
    n = 5 # nb joints
    m = 1 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    t_min = -np.ones(n)
    t_max = np.ones(n)
    robot.force_polytope(J, t_max, t_min)
    assert True

# unit test for force_polytope_intersection  function with random data 1d data
def test_force_polytope_intersection_random_1d():
    n = 5 # nb joints
    m = 1 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    J2 = np.random.rand(m,n)*2 - 1
    t_min = -np.ones(n)
    t_max = np.ones(n)
    robot.force_polytope_intersection(J, J2, t_max, t_min, t_max, t_min)
    assert True


# unit test for acceleration_polytope  function with random data and bias   
def test_acceleration_poly_random_bias():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    M = np.random.rand(n,n)
    t_min = np.zeros(n)
    t_max = np.ones(n)
    t_bias = np.zeros(n)
    robot.acceleration_polytope(J, M, t_min, t_max, t_bias)
    assert True

    

# unit test for reachable space approximation using random data
def test_approximation_reachable_space_random():
    n = 5 # nb joints
    m = 3 # cartesian forces
    
    J = np.random.rand(m,n)*2 - 1
    M = np.random.rand(n,n)

    q0 = np.random.rand(n)*2 - 1
    dq_max = np.ones(n)
    dq_min = -np.ones(n)
    q_min = -np.ones(n)
    q_max = np.ones(n)
    t_min = np.zeros(n)
    t_max = np.ones(n)
    t_h = 0.1
    
    robot.reachable_space_approximation(M,J,q0,horizon=t_h,t_max=t_max,t_min=t_min,dq_max=dq_max,dq_min=dq_min,q_min=q_min,q_max=q_max)
    assert True