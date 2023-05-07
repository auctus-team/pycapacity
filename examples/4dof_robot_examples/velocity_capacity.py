import numpy as np
import matplotlib.pyplot as plt

import pycapacity.robot as capacity # robot capacity module
import pycapacity.visual as visual # visualistion tools

from four_link_utils import *

#joint positions q
q  = np.random.rand(4)*np.pi/3*2-1
# joint torque limits tau
dq_min = -np.ones((4,1))
dq_max = np.ones((4,1))


# jacobian
J = four_link_jacobian(q)
# calculate the force polytope

vel_poly = capacity.velocity_polytope(J, dq_min ,dq_max)

# calculate the force ellipsoid
vel_ellipsoid = capacity.velocity_ellipsoid(J, dq_max)


# visualise polytope ellispoid
fig = plt.figure(13, figsize=[10,10])

scale = 1/5

#plot the robot
robot_position = four_link_forward_kinematics(q) 
four_link_plot_robot(plt, q)

#plot the polytope
visual.plot_polytope(plot=fig,
                    polytope=vel_poly,
                    center=robot_position, 
                    face_color='lightsalmon', 
                    edge_color='orangered',
                    vertex_color='gray',
                    label='polytope', 
                    scale=scale)
# plot ellispoid
visual.plot_ellipsoid(ellipsoid=vel_ellipsoid, 
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