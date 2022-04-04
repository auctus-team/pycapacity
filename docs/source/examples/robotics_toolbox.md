# Robotics toolbox  examples
![](https://github.com/petercorke/robotics-toolbox-python/raw/master/docs/figs/RobToolBox_RoundLogoB.png)


## Installing robotics toolbox

Robotics toolbox can be easily downloaded as sa pip package or our preferred way using anaconda.
### Pip package install
```
pip install roboticstoolbox-python
```

### Anaconda install

For anaconda instals you can simply download the yaml file and save it as `env.yaml`:
```yaml
name: rtbx_examples
channels:
  - conda-forge
dependencies:
  - python=3.8
  - roboticstoolbox-python
  - numpy
  - matplotlib
  - pip
  - pip:
    - pycapacity
    - trimesh
```
And create a new ready to go environment:
```
conda env create -f env.yaml    # create the new environemnt and install robotics toolbosx, pycapacity, ...
conda actiavte rtbx_examples
```

#### Creating the custom environment from scratch
You can also simply use anaconda to create a new custom environment:
```bash
conda create -n rtbx_examples python=3.8 # create python 3.8 based environment
conda activate rtbx_examples
conda install -c conda-forge roboticstoolbox-python numpy matplotlib 
```

Then install `pycapacity` for the workspace analysis
```bash
pip install pycapacity
```

## Code example using pyplot
Calculating the force polytope of the panda robot and visualising it using the pyplot.

```python
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
import pycapacity.robot as capacity_solver

# robot matrices
Jac = panda.jacob0(q)[:3,:]
tau = panda.rne(q, np.zeros((panda.n,)), np.zeros((panda.n,)))
gravity = panda.gravload(q).reshape((-1,1))
print(np.round(Jac,3),panda.qr,gravity,tau)
# calculate for ce polytope
vertices, faces_index =  capacity_solver.force_polytope_withfaces(Jac, t_max, t_min, gravity)
# end effector position
panda_ee_position = panda.fkine(q).t
# scale the polytiope and place it to the end-effector
scaling = 500
faces = capacity_solver.face_index_to_vertex(vertices/scaling + panda_ee_position[:, None],faces_index)


# plotting the polytope using pycapacity
import matplotlib.pyplot as plt
from pycapacity.visual import plot_polytope_faces, plot_polytope_vertex # pycapacity visualisation tools

# visualise panda
fig = panda.plot(q)
ax = fig.ax

# draw faces and vertices
plot_polytope_vertex(ax=ax, vertex=vertices, label='force polytope',color='blue')
plot_polytope_faces(ax=ax, faces=faces, face_color='blue', edge_color='blue', alpha=0.2)

ax.set_xlim([-1, 1.5])
ax.set_ylim([-1, 1.5])
ax.set_zlim([0, 1.5])
plt.tight_layout()
plt.legend()
plt.show()
fig.hold()
```
![](../images/rb_pyplot.png)

## Code example using Swift
Calculating the force polytope of the panda robot and visualising it using the Swift.

```python
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
import pycapacity.robot as capacity_solver

# robot matrices
Jac = panda.jacob0(q)[:3,:]
tau = panda.rne(q, np.zeros((panda.n,)), np.zeros((panda.n,)))
gravity = panda.gravload(q).reshape((-1,1))
print(np.round(Jac,3),panda.qr,gravity,tau)
# calculate for ce polytope
vertices, faces_index =  capacity_solver.force_polytope_withfaces(Jac, t_max, t_min, gravity)
faces = capacity_solver.face_index_to_vertex(vertices,faces_index)
# end effector position
panda_ee_position = panda.fkine(q).t

# visualise panda
panda = rp.models.Panda()
import swift.Swift as Swift
panda.q = q
env = Swift()
env.launch()
env.add(panda)

# polytope visualisaation
import trimesh
# save polytope as mesh file
scaling = 500
mesh = trimesh.Trimesh(vertices=(vertices.T/scaling + panda_ee_position) ,
                       faces=faces_index, use_embree=True, validate=True)
f = open("demofile.stl", "wb")
f.write(trimesh.exchange.stl.export_stl(mesh))
f.close()
# robot visualisaiton
from spatialgeometry import Mesh
poly_mesh = Mesh('demofile.stl')
poly_mesh.set_alpha(0.5)
env.add(poly_mesh)
```

![](../images/rb_swig.png)

## Code example using matplotlib
Calculating the force polytope of the panda robot only polytope using matplotlib

```python
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
import pycapacity.robot as capacity_solver

# robot matrices
Jac = panda.jacob0(q)[:3,:]
tau = panda.rne(q, np.zeros((panda.n,)), np.zeros((panda.n,)))
gravity = panda.gravload(q).reshape((-1,1))
print(np.round(Jac,3),panda.qr,gravity,tau)
# calculate for ce polytope
vertices, faces_index =  capacity_solver.force_polytope_withfaces(Jac, t_max, t_min, gravity)
# end effector position
panda_ee_position = panda.fkine(q).t
# scale the polytiope and place it to the end-effector
scaling = 500
vertices = vertices/scaling + panda_ee_position[:, None]
faces = capacity_solver.face_index_to_vertex(vertices,faces_index)

# plotting the polytope using pycapacity
import matplotlib.pyplot as plt
from pycapacity.visual import plot_polytope_faces, plot_polytope_vertex # pycapacity visualisation tools
fig = plt.figure()
# draw faces and vertices
ax = plot_polytope_vertex(plt=plt, vertex=vertices, label='force polytope',color='blue')
plot_polytope_faces(ax=ax, faces=faces, face_color='blue', edge_color='blue', alpha=0.2)

plt.tight_layout()
plt.legend()
plt.show()
```
![](../images/rb_matplotlib.png)