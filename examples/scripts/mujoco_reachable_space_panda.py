"""
Mujoco and pycapacity example

Calculating the reachable workspace within a horizon for panda robot and visualising it using mujoco.

We definitely recommend using a virtual environment for installing the dependencies, such as anaconda or virtualenv.
See the tutorial page for more information: https://auctus-team.github.io/pycapacity/examples/mujoco.html

Installing dependencies briefly:
- mujoco : pip install mujoco
- robot_descriptions : pip install robot_descriptions
- pycapacity : pip install pycapacity
- cgal: pip install cgal (optional)

"""

import mujoco
import mujoco.viewer
import numpy as np

from robot_descriptions.loaders.mujoco import load_robot_description
# Loading a variant of the model, e.g. panda without a gripper.
model = load_robot_description("panda_mj_description", variant="panda_nohand"); frame_name = "link7"

# Set the timestep to 20ms
model.opt.timestep = 0.02

# Create a data structure to hold the state of the robot
data = mujoco.MjData(model)

# force polytope calculation function
from pycapacity.robot import reachable_space_nonlinear
# visualisation utils from pycapacity package
import pycapacity.visual as pyviz

# get max and min joint torques
q_max = model.actuator_ctrlrange[:,1]
q_min = model.actuator_ctrlrange[:,0]
# velocity limits from here: https://frankaemika.github.io/docs/control_parameters
dq_max = np.array([2.1750, 2.1750,2.1750,2.1750,2.6100, 2.6100,2.6100])
dq_min = -dq_max

# Create a data structure to hold the state of the robot (only for the FK calculation)
data1 = mujoco.MjData(model)
# A simple forward kinematics (FK) function
def fk(q):
    data1.qpos[:7] = q
    mujoco.mj_kinematics(model, data1)
    return data1.site_xpos[0]


# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TEXTURE] = 0  # Disable textures
        
        # Compute the reachable space of the end-effector
        poly = reachable_space_nonlinear(
            forward_func= fk,
            q0 = data.qpos[:7],
            time_horizon= 0.2,
            q_min=q_min,
            q_max=q_max,
            dq_min=dq_min,
            dq_max=dq_max,
            options= {'calculate_faces':True, 'convex_hull': False, 'n_samples': 3, 'facet_dim': 1}
        )
    
        # Draw force polytope
        pyviz.mj_draw_polytope(viewer, poly, edges=True, faces=True) 
        
        # Update the viewer        
        viewer.sync()