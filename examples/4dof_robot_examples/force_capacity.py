import numpy as np
import matplotlib.pyplot as plt

import pycapacity.robot as capacity # robot capacity module
import pycapacity.visual as visual # visualistion tools

from four_link_utils import four_link_jacobian, four_link_robot_plot 

# joint positions q
q  = np.random.rand(4)*np.pi/3*2-1
# joint torque limits tau
tau_min = -np.ones((4,1))
tau_max = np.ones((4,1))

# find robot position
# a bit of scaling
robot_position = four_link_robot_plot(q)*5 

# jacobian
J = four_link_jacobian(q)
# calculate the velocity polytope
f_vert, faces_indices = capacity.force_polytope_withfaces(J,tau_min,tau_max)
faces = capacity.face_index_to_vertex(f_vert, faces_indices)

# calculate the velocity ellipsoid
S,U = capacity.force_ellipsoid(J, tau_max)

# visualise polytope ellispoid
fig = plt.figure(12)
ax = fig.gca()
#plot the robot
plt.plot(robot_position[0,:],robot_position[1,:],linewidth=5, label="robot")
plt.plot(robot_position[0,:],robot_position[1,:],'ko',linewidth=5)
#plot the polytope
visual.plot_polytope_faces(ax=ax,faces=faces,center=robot_position[:,-1], face_color='lightsalmon', edge_color='orangered',label='polytope')
visual.plot_polytope_vertex(ax=ax,vertex=f_vert,center=robot_position[:,-1], color='gray')
# plot ellispoid
visual.plot_ellipsoid(S, U, center=robot_position[:,-1], ax=ax, label='ellipsoid', color=None, edge_color='blue', alpha=1.0)

plt.title("Force capacity")
plt.grid()
plt.axis('equal')
plt.legend()
plt.show()