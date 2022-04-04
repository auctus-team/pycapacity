Randomized Human musculoskeletal models
==================

A simple example program for force polytope  evaluation of a randomised human musculoskeletal model. 
Simply change the number of dof, number of forces and force limits and see how the calculation time and shape evaluates.

```python
import pycapacity.human as capacity # robot capacity module
import numpy as np

L = 30 # number of muscles
m = 3 # 3d forces
n = 6 # number of joints - dof

J = np.array(np.random.rand(m,n))*2-1 # random jacobian matrix
N = np.array(np.random.rand(n,L))*2-1 # random moment arm matrix

F_max = 100*np.ones(L)  # muscle forces limits max and min
F_min = np.zeros(L)

vertices, H,d, face_indexes = capacity.force_polytope(J,N, F_min, F_max, 0.1) # calculate the polytope vertices and faces
faces = capacity.face_index_to_vertex(vertices, face_indexes)

print(vertices) # display the vertices

# plotting the polytope
import matplotlib.pyplot as plt
from pycapacity.visual import plot_polytope_faces, plot_polytope_vertex # pycapacity visualisation tools
fig = plt.figure(4)

# draw faces and vertices
ax = plot_polytope_vertex(plt=plt, vertex=vertices, label='force',color='blue')
plot_polytope_faces(ax=ax, faces=faces, face_color='blue', edge_color='blue', alpha=0.2)

plt.tight_layout()
plt.legend()
plt.show()

```
![](../images/rand_human.png)
