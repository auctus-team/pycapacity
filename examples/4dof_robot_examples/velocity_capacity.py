import numpy as np
import matplotlib.pyplot as plt

from pycapacity.robot import * # robot capacity module
from pycapacity.visual import * # visualistion tools

# four link robot import
from pycapacity.examples import FourLinkRobot

# create the robot
robot = FourLinkRobot()


# joint positions q
q  = np.random.rand(4)*np.pi/2
# joint torque limits tau
tau_min = -np.ones((4,1))
tau_max = np.ones((4,1))


# jacobian
J = robot.jacobian(q)
# jacobian
M = robot.inertia(q)

# calculate the velocity polytope
a_poly = acceleration_polytope(J, M, tau_min ,tau_max)

# calculate the velocity ellipsoid
a_ellipsoid = acceleration_ellipsoid(J, M, tau_max)


# visualise polytope ellipsoid
fig = plt.figure(13, figsize=[10,10])
scale = 1/50


# plot the robot
robot_position = robot.forward_kinematics(q) 
robot.plot(plt, q)

# plot the polytope
plot_polytope(plot=plt,
              polytope=a_poly,
              center=robot_position, 
              face_color='lightsalmon', 
              edge_color='orangered',
              vertex_color='gray',
              label='polytope', 
              scale=scale)

# plot ellipsoid
plot_ellipsoid(ellipsoid=a_ellipsoid, 
               center=robot_position, 
               plot=plt, 
               label='ellipsoid', 
               edge_color='blue', 
               alpha=1.0,
               scale=scale)

plt.title("Acceleration capacity")
plt.grid()
plt.axis('equal')
plt.legend()
plt.show()