
# Python real-time capable robot task-space capacity calculation module

![](https://gitlab.inria.fr/askuric/polytope_vertex_search/-/raw/master/images/rviz_screenshot_2020.png)

This module provides a framework for the generic robot capacity calculation called in a form of a python module `pycapacity`.

This module integrates several velocity, force and acceleration capacity calculation functions based on ellipsoids:
- Velocity (manipulability) ellipsoid <br> `E_vel = {dx | dx = J.dq, ||dq||<1 }`
- Acceleration (dynamic manipulability) ellipsoid <br> `E_acc = {ddx | ddx = J.M^(-1).t, ||t||<1 }`
- Force ellipsoid <br> `E_for = {f | J^T.f = t, ||t||<1 }`

And polytopes: 
- Velocity polytope <br> `P_vel = {dx | dx = J.dq,  dq_min < dq < dq_max}`
- Acceleration polytope <br> `P_acc = {ddx | ddx = J.M^(-1).t, t_min < t < t_max}`
- Force polytope <br> `P_for = {f | J^T.f = t, t_min < t < t_max}`
- Force polytopes *Minkowski sum and intersection*

The force polytope functions have been implemented according to the paper:<br>
[**On-line force capability evaluation based on efficient polytope vertex search**](https://arxiv.org/abs/2011.05226)<br> 
by Antun Skuric, Vincent Padois and David Daney<br> Published on ICRA2021

And the velocity and acceleration polytopes are resolved using the *Hyper-plane shifting method*:<br>
[**Characterization of Parallel Manipulator Available Wrench Set Facets**](http://www.lirmm.fr/krut/pdf/2010_gouttefarde_ark-0602650368/2010_gouttefarde_ark.pdf)<br>
by Gouttefarde M., Krut S. <br>In: Lenarcic J., Stanisic M. (eds) Advances in Robot Kinematics: Motion in Man and Machine. Springer, Dordrecht (2010)

## Installation

All you need to do to install it is:
```
pip install pycapacity
```
And include it to your python project
```python
import pycapacity
```

Other way to install the code is by installing it directly from the git repo:
```
pip install git+https://gitlab.inria.fr/askuric/pycapacity
```

# Installation to enable modifying the source
Clone or download the repository to your pc:
```
cd your/project/path
git clone https://gitlab.inria.fr/askuric/pycapacity.git
```
Put the folder in your project directory:
```
my_project:
|
├── your
├── project 
├── files
|
└── pycapacity
```
And then import it in your python code for example:
```python
import pycapacity.pycapacity as capacity
```
> Note: if you do not install the module using pip you will need to import it using double name `pycapacity.pycapacity`.

All the smarts is placed in the `pycapacity.py` file so that is where you will find all the implementations and where to go to modify the code.

## Module functions docs

See full docs at the [link](https://gitlab.inria.fr/askuric/pycapacity/-/tree/master/docs)

- [`acceleration_ellipsoid`](https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/docs/pycapacity.md#function-acceleration_ellipsoid): acceleration ellipsoid calculation (dynamic manipulability ellipsoid)
- [`acceleration_polytope`](https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/docs/pycapacity.md#function-acceleration_polytope): Acceleration polytope calculating function
- [`acceleration_polytope_withfaces`](https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/docs/pycapacity.md#function-acceleration_polytope_withfaces): Acceleration polytope calculating function
- [`force_ellipsoid`](https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/docs/pycapacity.md#function-force_ellipsoid): force manipulability ellipsoid calculation
- [`force_polytope`](https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/docs/pycapacity.md#function-force_polytope): Force polytope representing the capacities of the two robots in a certain configuration
- [`force_polytope_intersection`](https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/docs/pycapacity.md#function-force_polytope_intersection): Force polytope representing the intersection of the capacities of the two robots in certain configurations.
- [`force_polytope_intersection_withfaces`](https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/docs/pycapacity.md#function-force_polytope_intersection_withfaces): Force polytope representing the intersection of the capacities of the two robots in certain configurations.
- [`force_polytope_sum_withfaces`](https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/docs/pycapacity.md#function-force_polytope_sum_withfaces): Force polytope representing the minkowski sum of the capacities of the two robots in certain configurations.
- [`force_polytope_withfaces`](https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/docs/pycapacity.md#function-force_polytope_withfaces): Force polytope representing the capacities of the two robots in a certain configuration.
- [`velocity_ellipsoid`](https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/docs/pycapacity.md#function-velocity_ellipsoid): velocity manipulability ellipsoid calculation
- [`velocity_polytope`](https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/docs/pycapacity.md#function-velocity_polytope): Velocity polytope calculating function
- [`velocity_polytope_withfaces`](https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/docs/pycapacity.md#function-velocity_polytope_withfaces): Velocity polytope calculating function, with faces


Algorithms:
- [`hyper_plane_shift_method`](https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/docs/pycapacity.md#function-hyper_plane_shift_method): Hyper plane shifting method implementation used to solve problems of a form:
---

## Code examples


```python
"""
A simple example program 3d force polytope 
evaluation of a randomised 6dof robot 
"""
from pycapacity import force_polytope_withfaces as polytope
import numpy as np

m = 3 # 3d forces
n = 6 # robot dof

J = np.array(np.random.rand(m,n)) # random jacobian matrix

t_min = np.ones((n,1))  # joint torque limits max and min
t_max = -np.ones((n,1))

vertices, faces = polytope(J,t_min, t_max) # calculate the polytope vertices and faces

print(vertices) # display the vertices
```

See [`demo_notebook.ipynb`](https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/demo_notebook.ipynb) for one example use case of the module.