"""
Mujoco and pycapacity example

Calculating the force polytope different robots and visualising it using mujoco.

We definitely recommend using a virtual environment for installing the dependencies, such as anaconda or virtualenv.
See the tutorial page for more information: https://auctus-team.github.io/pycapacity/examples/mujoco.html

Installing dependencies briefly:
- mujoco : pip install mujoco
- robot_descriptions : pip install robot_descriptions
- pycapacity : pip install pycapacity

"""

import mujoco
import mujoco.viewer
import numpy as np

from robot_descriptions.loaders.mujoco import load_robot_description
# Loading a variant of the model, e.g. panda without a gripper.
# model = load_robot_description("xarm7_mj_description", variant="xarm7_nohand"); frame_name = "link7"
# model = load_robot_description("iiwa14_mj_description"); frame_name = "link7"
# model = load_robot_description("gen3_mj_description"); frame_name = "forearm_link"
model = load_robot_description("panda_mj_description", variant="panda_nohand"); frame_name = "link7"
# model = load_robot_description("ur10e_mj_description"); frame_name = "wrist_3_link"
# model = load_robot_description("ur5e_mj_description"); frame_name = "wrist_3_link"
# model = load_robot_description("fr3_mj_description"); frame_name = "fr3_link6"

# Create a data structure to hold the state of the robot
data = mujoco.MjData(model)

# force polytope calculation function
from pycapacity.robot import force_polytope
# visualisation utils from pycapacity package
import pycapacity.visual as pyviz

# get max and min joint torques
tau_max = model.actuator_forcerange[:,1]
if np.any(tau_max <= 0):
    print("Warning: Negative or zero torque limits detected, using default value of 10 Nm")
    tau_max = np.ones_like(tau_max)*10
tau_min = -tau_max


# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TEXTURE] = 0  # Disable textures
        # Compute the Jacobian of the end-effector
        J_pos = np.zeros((3, model.nv))  # Linear Jacobian (3xnq)
        J_rot = np.zeros((3, model.nv))  # Rotational Jacobian (3xnq)
        mujoco.mj_jacBodyCom(model, data, J_pos, J_rot, model.body(frame_name).id)
        J = J_pos  # Use only the linear Jacobian
        
        # Compute the force polytope
        poly = force_polytope(J, tau_min, tau_max)
        # Shift polytope to the current end-effector position
        # and scale it for easier visualization
        poly.vertices = poly.vertices / 500 + data.site_xpos[0][:,None]
        # Draw force polytope
        pyviz.mj_draw_polytope(viewer, poly, edges=True, faces=True) 
        
        # Update the viewer        
        viewer.sync()