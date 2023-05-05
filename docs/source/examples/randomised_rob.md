Randomized robot
==================

A simple example program for force polytope  evaluation of a randomised robot model. 
Simply change the number of dof torque limits and see how the calculation time and shape evaluates.


```python
import pycapacity.robot as capacity # robot capacity module
import numpy as np

m = 3 # 3d forces
n = 6 # robot dof 

J = np.array(np.random.rand(m,n)) # random jacobian matrix

t_max = np.ones(n)  # joint torque limits max and min
t_min = -np.ones(n)

f_poly = capacity.force_polytope(J,t_min, t_max) # calculate the polytope

print(f_poly.vertices) # display the vertices

# plotting the polytope
import matplotlib.pyplot as plt
from pycapacity.visual import * # pycapacity visualisation tools
fig = plt.figure(4)

# draw faces and vertices
plot_polytope(plot=plt, 
            polytope=f_poly, 
            label='force', 
            edge_color='black', 
            alpha = 0.4)

plt.legend()
plt.show()
```
![](../images/rand_rob_matplotlib.png)
