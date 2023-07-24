import numpy as np
import matplotlib.pyplot as plt

import pycapacity.robot as capacity # robot capacity module
import pycapacity.visual as visual # visualistion tools

from four_link_utils import *

#joint positions q
q  = np.random.rand(4)*np.pi/2
# joint torque limits tau
tau_min = -np.ones((4,1))
tau_max = np.ones((4,1))


# jacobian
J = four_link_jacobian(q)
# jacobian
M = four_link_inertia(q)

# calculate the acceleration polytope

acc_poly = capacity.acceleration_polytope(J, M, tau_min ,tau_max)

# calculate the acceleration ellipsoid
acc_ellisoid= capacity.acceleration_ellipsoid(J, M, tau_max)

# visualise polytope ellipsoid
fig = plt.figure(13, figsize=[10,10])

scale = 1/100

#plot the robot
robot_position = four_link_forward_kinematics(q) 
four_link_plot_robot(plt, q)

#plot the polytope
visual.plot_polytope(plot=fig,
                    polytope=acc_poly,
                    center=robot_position, 
                    face_color='lightsalmon', 
                    edge_color='orangered',
                    vertex_color='gray',
                    label='polytope', 
                    scale=scale)
# plot ellipsoid
visual.plot_ellipsoid(ellipsoid=acc_ellisoid, 
               center=robot_position, 
               plot=fig,
               label='ellipsoid', 
               edge_color='blue', 
               alpha=1.0, 
               scale=scale)
plt.grid()
plt.legend()
plt.axis('equal')
plt.show()