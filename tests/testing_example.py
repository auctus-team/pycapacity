from pycapacity.examples import FourLinkRobot
import numpy as np

# test robot forward kinematics
def test_forward_kinematics():
    robot = FourLinkRobot()
    q = np.random.rand(4)*np.pi/3*2-1
    robot.forward_kinematics(q)
    assert True

#test the inertia matrix
def test_inertia():
    robot = FourLinkRobot()
    q = np.random.rand(4)*np.pi/3*2-1
    robot.inertia(q)
    assert True

# test the jacobian matrix
def test_jacobian():
    robot = FourLinkRobot()
    q = np.random.rand(4)*np.pi/3*2-1
    robot.jacobian(q)
    assert True