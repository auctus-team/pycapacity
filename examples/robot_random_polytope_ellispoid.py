import numpy as np
import matplotlib.pyplot as plt

import pycapacity.robot as capacity # robot capacity module
import pycapacity.visual as visual # visualistion tools


# random robot model
m = 3 # 3d forces
n = 6 # robot dof 
J = np.array(np.random.rand(m,n))

# torque limits
t_min = -10*np.ones(n)
t_max = 10*np.ones(n)


# force polytope calculation
f_vert, faces_indices = capacity.force_polytope_withfaces(J,t_max,t_min)
faces_force = capacity.face_index_to_vertex(f_vert, faces_indices)

# force ellispoid calcualtion
S,U = capacity.force_ellipsoid(J, t_max)

# visualisaiton
fig = plt.figure(2)

# plot polygones
ax = visual.plot_polytope_faces(plt=plt, faces=faces_force, face_color='blue', edge_color='black',label='polytope',alpha=0.2)
visual.plot_polytope_vertex(ax=ax, vertex=f_vert,color='blue')
    
# plot ellipsoid
visual.plot_ellipsoid(S, U, ax=ax, label='ellipsoid', color='orange',alpha=0.5)

plt.title('Force capacity for random matrices')
plt.tight_layout()
plt.legend()
plt.show()