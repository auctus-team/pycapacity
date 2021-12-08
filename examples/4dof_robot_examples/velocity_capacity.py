import numpy as np
import matplotlib.pyplot as plt

import pycapacity.robot as capacity # robot capacity module
import pycapacity.visual as visual # visualistion tools

from four_link_utils import four_link_jacobian, four_link_robot_plot 

#joint positions q
q  = np.random.rand(4)*np.pi/3*2-1
# joint torque limits tau
dq_min = -np.ones((4,1))
dq_max = np.ones((4,1))

# find robot position
# a bit of scaling
robot_position = four_link_robot_plot(q)*5

# jacobian
J = four_link_jacobian(q)
# calculate the force polytope
vel_vert, faces_indices = capacity.velocity_polytope_withfaces(J, dq_min ,dq_max)
faces = capacity.face_index_to_vertex(vel_vert, faces_indices)

# calculate the force ellipsoid
S,U = capacity.velocity_ellipsoid(J, dq_max)


# visualise polytope ellipsoid
fig = plt.figure(14)
ax = plt.gca()

#plot the robot
plt.plot(robot_position[0,:],robot_position[1,:],linewidth=5, label="robot")
plt.plot(robot_position[0,:],robot_position[1,:],'ko',linewidth=5)
#plot the polytope
visual.plot_polytope_faces(ax=plt,faces=faces,center=robot_position[:,-1],face_color='lightsalmon', edge_color='orangered',label='polytope')
visual.plot_polytope_vertex(ax=ax,vertex=vel_vert,center=robot_position[:,-1],color='gray')
# plot the ellipsoid
visual.plot_ellipsoid(S, U, center=robot_position[:,-1], ax=ax, label='ellipsoid', color=None, edge_color='blue', alpha=1.0)

plt.title("Velocity capacity")
plt.grid()
plt.legend()
plt.show()