import numpy as np
import matplotlib.pyplot as plt

from pycapacity.robot import * # robot capacity module
from pycapacity.visual import * # visualistion tools

# four link robot import
from pycapacity.examples import FourLinkRobot

# create the robot
robot = FourLinkRobot()

# joint positions q
q  = np.random.rand(4)*np.pi/3*2-1
# joint torque limits tau
tau_min = -np.ones((4,1))
tau_max = np.ones((4,1))

# jacobian
J = robot.jacobian(q)
# calculate the force polytope
f_poly = force_polytope(J,tau_min,tau_max)

# calculate the force ellipsoid
f_ellipsoid = force_ellipsoid(J, tau_max)

# visualise polytope ellipsoid
fig = plt.figure(12, figsize=[10,10])
scale = 1/5

# plot the robot
robot_position = robot.forward_kinematics(q) 
robot.plot(plt, q)

# plot the polytope
plot_polytope(plot=plt,
              polytope=f_poly,
              center=robot_position, 
              face_color='lightsalmon', 
              edge_color='orangered',
              vertex_color='gray',
              label='polytope', 
              scale=scale)

# plot ellipsoid
plot_ellipsoid(ellipsoid=f_ellipsoid, 
               center=robot_position, 
               plot=plt, 
               label='ellipsoid', 
               edge_color='blue', 
               alpha=1.0,
               scale=scale)
plt.title('Force capacity')
plt.grid()
plt.axis('equal')
plt.legend()
plt.show()