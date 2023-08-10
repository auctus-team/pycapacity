"""

Robotics Toolbox for Python: Plotting a force polytope using Swift

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
# calculate the face representation of the polytope
f_poly.find_faces()

 # visualise panda
panda = rp.models.Panda()
import swift.Swift as Swift
panda.q = q
env = Swift()
env.launch()
env.add(panda)


# polytope visualisation
import trimesh
# save polytope as mesh file
scaling = 500
# create the mesh
mesh = trimesh.Trimesh(vertices=(f_poly.vertices.T/scaling + panda.fkine(q).t),
                       faces=f_poly.face_indices, use_embree=True, validate=True)

# absolute path to the temporary polytope file saved 
# in the stl format
import os
file_path = os.path.join(os.getcwd(),'tmp_polytope_file.stl')
                  
f = open(file_path, "wb")
f.write(trimesh.exchange.stl.export_stl(mesh))
f.close()
# robot visualisation
from spatialgeometry import Mesh
poly_mesh = Mesh(file_path)
poly_mesh.color = (0.9,0.6,0.0,0.5)
env.add(poly_mesh)