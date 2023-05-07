import numpy as np
import matplotlib.pyplot as plt

import pycapacity.robot as capacity # robot capacity module
import pycapacity.visual as visual # visualistion tools

from four_link_utils import *

# joint positions q
q  = np.random.rand(4)*np.pi/3*2-1
# joint torque limits tau
tau_min = -np.ones((4,1))
tau_max = np.ones((4,1))

# jacobian
J = four_link_jacobian(q)
# calculate the velocity polytope
f_poly = capacity.force_polytope(J,tau_min,tau_max)

# calculate the velocity ellipsoid
f_ellipsoid = capacity.force_ellipsoid(J, tau_max)

# visualise polytope ellispoid
fig = plt.figure(12, figsize=[10,10])

scale = 1/5

#plot the robot
robot_position = four_link_forward_kinematics(q) 
four_link_plot_robot(plt, q)

#plot the polytope
visual.plot_polytope(plot=fig,
              polytope=f_poly,
              center=robot_position, 
              face_color='lightsalmon', 
              edge_color='orangered',
              vertex_color='gray',
              label='polytope', 
              scale=scale)

# plot ellispoid
visual.plot_ellipsoid(ellipsoid=f_ellipsoid, 
               center=robot_position, 
               plot=fig, 
               label='ellipsoid', 
               edge_color='blue', 
               alpha=1.0,
               scale=scale)

plt.grid()
plt.axis('equal')
plt.legend()
plt.show()