"""
A simple example program for 3d force polytope 
evaluation of a randomised 6dof robot 
"""
import pycapacity.robot as capacity # robot capacity module
import numpy as np

m = 3 # 3d forces
n = 6 # robot dof

J = np.array(np.random.rand(m,n)) # random jacobian matrix

t_max = np.ones(n)  # joint torque limits max and min
t_min = -np.ones(n)

vertices, face_indexes = capacity.force_polytope_withfaces(J,t_min, t_max) # calculate the polytope vertices and faces
faces = capacity.face_index_to_vertex(vertices, face_indexes)

print(vertices) # display the vertices

# plotting the polytope
import matplotlib.pyplot as plt
from pycapacity.visual import plot_polytope_faces, plot_polytope_vertex # pycapacity visualisation tools
fig = plt.figure(4)

# draw faces and vertices
ax = plot_polytope_vertex(plt=plt, vertex=vertices, label='force polytope',color='blue')
plot_polytope_faces(ax=ax, faces=faces, face_color='blue', edge_color='blue', alpha=0.2)

plt.tight_layout()
plt.legend()
plt.show()