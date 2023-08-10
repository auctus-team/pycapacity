"""

Robotics Toolbox for Python: Plotting a force polytope

This script shows how to plot a force polytope for a given robot configuration, it uses Panda robot as an example.
It visualises the polytope using pycapacity module.

We strongly recommend using a virtual environment for installing the dependencies, such as anaconda or virtualenv.
See the tutorial page for more information: https://auctus-team.github.io/pycapacity/examples/robotics_toolbox.html

Installing dependencies briefly:
- roboticstoolbox-python : pip install roboticstoolbox-python
- pycapacity : pip install pycapacity

"""

import roboticstoolbox as rp
import numpy as np

panda = rp.models.DH.Panda()
# initial pose
q= np.array([0.00138894 ,5.98736e-05,-0.30259058,   -1.6, -6.64181e-05,    1.56995,-5.1812e-05])
panda.q = q
# joint torque limits
t_max = np.array([87, 87, 87, 87, 20, 20, 20]) 
t_min = -t_max

# polytope python module
import pycapacity.robot as pyc

# robot matrices
Jac = panda.jacob0(q)[:3,:]
# gravity torque
gravity = panda.gravload(q).reshape((-1,1))

# calculate for the polytope
f_poly =  pyc.force_polytope(Jac, t_max, t_min, gravity)

# plotting the polytope using pycapacity
import matplotlib.pyplot as plt
from pycapacity.visual import * # pycapacity visualisation tools

# visualise panda
fig = panda.plot(q)
ax = fig.ax

# draw faces and vertices
plot_polytope(plot=plt, 
              polytope=f_poly, 
              label='force polytope',
              edge_color='black', 
              alpha = 0.2, 
              show_vertices=False,
              center=panda.fkine(q).t,  # set the polytope center at the end effector position
              scale=1/500) # scale the polytope and place it to the end-effector

ax.set_xlim([-1, 1.5])
ax.set_ylim([-1, 1.5])
ax.set_zlim([0, 1.5])
plt.legend()
plt.show()
fig.hold()