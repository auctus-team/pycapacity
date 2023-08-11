"""
Example of using Biorbd library with pycapacity and ploting the polytope in bioviz (in real time)
-------------------------------------------------------------------------------------------------

In order to install all the necessary dependencies, please follow the instructions on the pycapacity website:
https://auctus-team.github.io/pycapacity/examples/pyomeca.html
Even though it is possible to install all the dependencies from source and avoid using a virtual environment, we recommend using conda for the installation of the dependencies.


The example uses the MOBL_ARMS_fixed_33 model from pyomeca_models. You can download pyomeca_models using terminal:
```bash
git clone https://gitlab.inria.fr/auctus-team/components/modelisation/humanmodels/pyomeca_models.git
```
Or downloading the zip from the link 
https://gitlab.inria.fr/auctus-team/components/modelisation/humanmodels/pyomeca_models

Make sure to place the `pyomeca_models` folder in the directory that will contain your python code.


"""
import numpy as np
import biorbd
import bioviz
from bioviz.biorbd_vtk import VtkModel, VtkWindow, Mesh

# a bit of statistics
import time

# polytope algorithm
from pycapacity.human import force_polytope

# Load a predefined model
model = biorbd.Model("pyomeca_models/MOBL_ARMS_fixed_33.bioMod")

# get the number of dof and muslces
nq = model.nbQ()
nb_mus = model.nbMuscles()

# Animate the results if biorbd viz is installed
b = bioviz.Viz(loaded_model=model,
               background_color=(1,1,1), 
               show_local_ref_frame=False, 
               show_global_ref_frame=False, 
               show_markers=True,
               show_global_center_of_mass=False,
               show_segments_center_of_mass=False, 
               show_wrappings=False, 
               show_floor=False, 
               show_gravity_vector=False)
# define the meshes for the polytope - without robot
vtkMeshView = VtkModel(b.vtk_window, patch_color=[[0,0.5,0.8]],mesh_opacity=0.5)
vtkMeshView1 = VtkModel(b.vtk_window, patch_color=[[0,0.5,0.8]],mesh_opacity=0.8, force_wireframe=True)

b.set_q([0.0,1.4237,-1.256,1.8218,0.0,0.0,0.0])
while b.vtk_window.is_active:
    Q = b.Q
    model.updateMuscles(Q, True)
    model.UpdateKinematicsCustom(Q, np.zeros(nq), np.zeros(nq))

    F_max = []
    F_min = []
    for i in range(nb_mus):
        F_max.append(model.muscle(i).characteristics().forceIsoMax())
        #F_min.append(0)
        a = biorbd.HillThelenType(model.muscle(i)).FlPE()
        if a > 1:
            a = 0.1
        elif a < 0:
            a = 0
        F_min.append(a*F_max[-1])

    start = time.time()
    N = -model.musclesLengthJacobian(Q).to_array().T
    J = model.markersJacobian(Q, False, False)[-1].to_array()
    print("matrices time", time.time() - start)

    # Proceed with the inverse dynamics
    Tau_grav = model.InverseDynamics(Q, np.zeros(nq), np.zeros(nq))
    
    start = time.time()
    f_poly = force_polytope(J, N, F_min, F_max, 10, -Tau_grav.to_array())
    print("polytope time", time.time() - start)

    ## display polytope in the bioviz
    f_vert_show = f_poly.vertices/2000 + model.markers(Q)[model.nbMarkers()-1].to_array()[:,None]

    # plot polytope (blue) - with the edges
    meshes = []
    meshes.append(Mesh(vertex=f_vert_show[:,:,None], triangles=f_poly.face_indices.T))
    vtkMeshView.new_mesh_set(meshes)
    vtkMeshView1.new_mesh_set(meshes)

    # update visualisation
    b.update()
