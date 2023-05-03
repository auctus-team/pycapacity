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
    robot.acceleration_polytope(J, M, t_min, t_max)
    robot.velocity_polytope(J, dq_min, dq_max)


# unit test for force_polytope fucntion with randome data   
def test2():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    t_min = np.zeros(n)
    t_max = np.ones(n)
    robot.force_polytope(J, t_min, t_max)
    assert True

# unit test for velocity_polytope fucntion with randome data
def test3():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    dq_max = np.ones(n)
    robot.velocity_ellipsoid(J,  dq_max)
    assert True

# unit test for force_ellipsoid fucntion with randome data
def test4():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    t_max = np.ones(n)
    robot.force_ellipsoid(J, t_max)
    assert True

# unit test for acceleration_ellipsoid fucntion with randome data
def test5():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    M = np.random.rand(n,n)
    t_max = np.ones(n)
    robot.acceleration_ellipsoid(J, M, t_max)
    assert True

# unit test for force_polytope_intersection fucntion with randome data
def test6():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    J2 = np.random.rand(m,n)*2 - 1
    t_min = -np.ones(n)
    t_max = np.ones(n)
    robot.force_polytope_intersection(J, J2, t_max, t_min, t_max, t_min)
    assert True

# unit test for force_polytope_intersection fucntion with randome data with t_bias
def test6():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    J2 = np.random.rand(m,n)*2 - 1
    t_min = -np.ones(n)
    t_max = np.ones(n)
    t_bias = np.zeros(n)
    robot.force_polytope_intersection(J, J2, t_max, t_min, t_max, t_min, t_bias, t_bias)
    assert True

# unit test for force_polytope_sum fucntion with randome data
def test7():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    J2 = np.random.rand(m,n)*2 - 1
    t_min = -np.ones(n)
    t_max = np.ones(n)
    robot.force_polytope_sum(J, J2, t_max, t_min, t_max, t_min)
    assert True

# unit test for force_polytope fucntion with randome data 1d matrix J 
def test8():
    n = 5 # nb joints
    m = 1 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    t_min = -np.ones(n)
    t_max = np.ones(n)
    robot.force_polytope(J, t_max, t_min)
    assert True

# unit test for force_polytope_intersection fucntion with randome data 1d data
def test16():
    n = 5 # nb joints
    m = 1 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    J2 = np.random.rand(m,n)*2 - 1
    t_min = -np.ones(n)
    t_max = np.ones(n)
    robot.force_polytope_intersection(J, J2, t_max, t_min, t_max, t_min)
    assert True


# unit test for acceleration_polytope fucntion with randome data and bias   
def test9():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    M = np.random.rand(n,n)
    t_min = np.zeros(n)
    t_max = np.ones(n)
    t_bias = np.zeros(n)
    robot.acceleration_polytope(J, M, t_min, t_max, t_bias)
    assert True


# unit test for acceleration_polytope fucntion with randome data and bias   
def test10():
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    M = np.random.rand(n,n)
    t_min = np.zeros(n)
    t_max = np.ones(n)
    t_bias = np.zeros(n)
    robot.acceleration_polytope(J, M, t_min, t_max, t_bias)
    assert True
    