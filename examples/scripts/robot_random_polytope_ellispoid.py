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
f_poly =  capacity.force_polytope(J,t_max,t_min)

# force ellipsoid calculation
f_ellipsoid = capacity.force_ellipsoid(J, t_max)

# visualisaiton
fig = plt.figure(2)

# plot polygones
# plot polytope
visual.plot_polytope(plot=plt, polytope=f_poly, label='polytope', vertex_color='blue', edge_color='blue', alpha=0.2)

# plot ellipsoid
visual.plot_ellipsoid(ellipsoid=f_ellipsoid, plot=plt, label='ellipsoid', color='orange',alpha=0.5)

plt.title('Force capacity for random matrices')
plt.tight_layout()
plt.legend()
plt.show()