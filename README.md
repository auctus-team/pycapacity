
# Real-time capable task-space capacity calculation python module

<img src="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/raw/master/images/comparison.gif" height="300px">
<img src="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/raw/master/images/bimanual1.png" height="200px">

The `pycapacity` package provides a framework for the generic task-space capacity calculation for:
- Robotic serial manipulators - `pycapacity.robot`
- Human musculoskeletal models - `pycapacity.human`

This package also provides a module `pycapacity.polyalgos` with a set of polytope evaluation algorithms for standard polytope formulations, that can be used as a standalone library.

## Robotic manipulator capacity metrics
<img src='https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/raw/master/images/robot.png' height='300px'>

For the robotic manipulators the package integrates several velocity, force and acceleration capacity calculation functions based on ellipsoids:
- Velocity (manipulability) ellipsoid <br> `E_vel = {dx | dx = J.dq, ||dq||<1 }`
- Acceleration (dynamic manipulability) ellipsoid <br> `E_acc = {ddx | ddx = J.M^(-1).t, ||t||<1 }`
- Force ellipsoid <br> `E_for = {f | J^T.f = t, ||t||<1 }`

And polytopes: 
- Velocity polytope <br> `P_vel = {dx | dx = J.dq,  dq_min < dq < dq_max}`
- Acceleration polytope <br> `P_acc = {ddx | ddx = J.M^(-1).t, t_min < t < t_max}`
- Force polytope <br> `P_for = {f | J^T.f = t, t_min < t < t_max}`
- Force polytopes *Minkowski sum and intersection*

Where `J` is the robot jacobian matrix, `f` is the vector of cartesian forces,`dx` and `ddx` are vectors fo cartesian velocities and accretions, `dq` is the vector of the joint velocities and `t` is the vector of joint torques.

The force polytope functions have been implemented according to the paper:<br>
[**On-line force capability evaluation based on efficient polytope vertex search**](https://arxiv.org/abs/2011.05226)<br> 
by A.Skuric, V.Padois and D.Daney<br> Published on ICRA2021

And the velocity and acceleration polytopes are resolved using the *Hyper-plane shifting method*:<br>
[**Characterization of Parallel Manipulator Available Wrench Set Facets**](http://www.lirmm.fr/krut/pdf/2010_gouttefarde_ark-0602650368/2010_gouttefarde_ark.pdf)<br>
by Gouttefarde M., Krut S. <br>In: Lenarcic J., Stanisic M. (eds) Advances in Robot Kinematics: Motion in Man and Machine. Springer, Dordrecht (2010)

## Human musculoskeletal models capacity metrics
<img src='https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/raw/master/images/force.png' height='200px'>

For the human musculoskeletal models this package implements the polytope metrics:
- Velocity polytope <br> `P_vel = {dx | dx = J.dq, dl = L.dq  dl_min < dl < dl_max}`
- Acceleration polytope <br> `P_acc = {ddx | ddx = J.M^(-1).N.F, F_min < F < F_max}`
- Force polytope <br> `P_for = {f | J^T.f = N.F, F_min < F < F_max}`

Where `J` is the model's jacobian matrix, `L` si the muscle length jacobian matrix, `N= -L^T` is the moment arm matrix, `f` is the vector of cartesian forces,`dx` and `ddx` are vectors fo cartesian velocities and accretions, `dq` is the vector of the joint velocities, `t` is the vector of joint torques, `dl` is the vector of the muscle stretching velocities and `F` is the vector of muscular forces. 

The force and velocity polytope functions have been implemented according to the paper:<br>
[**On-line feasible wrench polytope evaluation based on human musculoskeletal models: an iterative convex hull method**](https://hal.inria.fr/hal-03369576)<br> 
by A.Skuric, V.Padois, N.Rezzoug and D.Daney<br> Submitted to RAL & ICRA2022 

And the acceleration polytopes are resolved using the *Hyper-plane shifting method*:<br>
[**Characterization of Parallel Manipulator Available Wrench Set Facets**](http://www.lirmm.fr/krut/pdf/2010_gouttefarde_ark-0602650368/2010_gouttefarde_ark.pdf)<br>
by Gouttefarde M., Krut S. <br>In: Lenarcic J., Stanisic M. (eds) Advances in Robot Kinematics: Motion in Man and Machine. Springer, Dordrecht (2010)

## Polytope evaluation algorithms

There are three methods implemented in this paper to resolve all the polytope calculations:
- Hyper-plane shifting method
- Iterative convex hull method
- Vertex enumeration auctus

All of the methods are implemented in the module `pycapacity.polyalgos` and can be used as standalone functions.  See in [docs for more info](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/polyalgos.md). 

### Hyper-plane shifting method
[**Characterization of Parallel Manipulator Available Wrench Set Facets**](http://www.lirmm.fr/krut/pdf/2010_gouttefarde_ark-0602650368/2010_gouttefarde_ark.pdf)<br>
by Gouttefarde M., Krut S. <br>In: Lenarcic J., Stanisic M. (eds) Advances in Robot Kinematics: Motion in Man and Machine. Springer, Dordrecht (2010)

This method finds the half-space representation of the polytope of a class:
```
P = {x | x = By, y_min <= y <= y_max }
```
To find the vertices of the polytope after finding the half-space representation `Hx <= d` an convex-hull algorithm is used. 

The method is a part of the `pycapacity.polyalgos` module `hyper_plane_shift_method`, See in [docs for more info](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/polyalgos.md#function-hyper_plane_shift_method). 

### Iterative convex-hull method
[**On-line feasible wrench polytope evaluation based on human musculoskeletal models: an iterative convex hull method**](https://hal.inria.fr/hal-03369576)<br> 
by A.Skuric, V.Padois, N.Rezzoug and D.Daney<br> Submitted to RAL & ICRA2022 

This method finds both vertex and half-space representation of the class of polytopes:
```
P = {x | Ax = By, y_min <= y <= y_max }
``` 
And it can be additionally extended to the case where there is an additional projection matrix `P` making a class of problems:
```
P = {x | x= Pz, Az = By, y_min <= y <= y_max }
``` 

The method is a part of the `pycapacity.polyalgos` module `iterative_convex_hull_method`. See the [docs for more info](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/polyalgos.md#function-iterative_convex_hull_method)

### Vertex enumeration auctus
[**On-line force capability evaluation based on efficient polytope vertex search**](https://arxiv.org/abs/2011.05226)<br> 
by A.Skuric, V.Padois and D.Daney<br> Published on ICRA2021

This method finds vertex representation of the class of polytopes:
```
P = {x | Ax = y, y_min <= y <= y_max }
``` 
To find the half-space representation (faces) of the polytope after finding the vertex representation  an convex-hull algorithm is used. 

The method is a part of the `pycapacity.polyalgos` module `vertex_enumeration_auctus`. See the [docs for more info](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/polyalgos.md#function-vertex_enumeration_auctus)

## Installation

All you need to do to install it is:
```
pip install pycapacity
```
And include it to your python project
```python
import pycapacity.robot 
# and/or
import pycapacity.human 
#and/or
import pycapacity.polyalgos  as algos
```

Other way to install the code is by installing it directly from the git repo:
```
pip install git+https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/
```

## Package API docs

See full docs at the [link](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/tree/master/docs)

### Modules

- [`human`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/human.md#module-human)
- [`polyalgos`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/polyalgos.md#module-polyalgos)
- [`robot`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/robot.md#module-robot)

### Functions

Robot metrics
- [`robot.acceleration_ellipsoid`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/robot.md#function-acceleration_ellipsoid): acceleration ellipsoid calculation (dynamic manipulability ellipsoid)
- [`robot.acceleration_polytope`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/robot.md#function-acceleration_polytope): Acceleration polytope calculating function
- [`robot.acceleration_polytope_withfaces`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/robot.md#function-acceleration_polytope_withfaces): Acceleration polytope calculating function
- [`robot.force_ellipsoid`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/robot.md#function-force_ellipsoid): force manipulability ellipsoid calculation
- [`robot.force_polytope`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/robot.md#function-force_polytope): Force polytope representing the capacities of the two robots in a certain configuration
- [`robot.force_polytope_intersection`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/robot.md#function-force_polytope_intersection): Force polytope representing the intersection of the capacities of the two robots in certain configurations.
- [`robot.force_polytope_intersection_withfaces`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/robot.md#function-force_polytope_intersection_withfaces): Force polytope representing the intersection of the capacities of the two robots in certain configurations.
- [`robot.force_polytope_sum_withfaces`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/robot.md#function-force_polytope_sum_withfaces): Force polytope representing the minkowski sum of the capacities of the two robots in certain configurations.
- [`robot.force_polytope_withfaces`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/robot.md#function-force_polytope_withfaces): Force polytope representing the capacities of the two robots in a certain configuration.
- [`robot.velocity_ellipsoid`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/robot.md#function-velocity_ellipsoid): velocity manipulability ellipsoid calculation
- [`robot.velocity_polytope`](https://gitlab.inria.fr/e capable task-space capacity calculation moduleauctus-team/people/antunskuric/pycapacity/-/blob/master/docs/robot.md#function-velocity_polytope): Velocity polytope calculating function
- [`robot.velocity_polytope_withfaces`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/robot.md#function-velocity_polytope_withfaces): Velocity polytope calculating function, with faces

Human metrics
- [`human.acceleration_polytope`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/human.md#function-acceleration_polytope): A function calculating the polytopes of achievable accelerations
- [`human.force_polytope`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/human.md#function-force_polytope): A function calculating the polytopes of achievable foreces based 
- [`human.joint_torques_polytope`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/human.md#function-joint_torques_polytope): A function calculating the polytopes of achievable joint torques
- [`human.torque_to_muscle_force`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/human.md#function-torque_to_muscle_force): A function calculating muscle forces needed to create the joint torques tau
- [`human.velocity_polytope`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/human.md#function-velocity_polytope): A function calculating the  polytopes of achievable velocity based 


Algorithms
- [`polyalgos.hyper_plane_shift_method`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/polyalgos.md#function-hyper_plane_shift_method): Hyper plane shifting method implementation used to solve problems of a form:
- [`polyalgos.iterative_convex_hull_method`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/polyalgos.md#function-iterative_convex_hull_method): A function calculating the polytopes of achievable x for equations form:
- [`polyalgos.make_2d`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/polyalgos.md#function-make_2d): Take a list of 3D(cooplanar) points and make it 2D
- [`polyalgos.make_unique`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/polyalgos.md#function-make_unique): Remove repetitions of columns
- [`polyalgos.order_index`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/polyalgos.md#function-order_index): Order clockwise 2D points
- [`polyalgos.stack`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/polyalgos.md#function-stack)
- [`polyalgos.vertex_enumeration_auctus`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/docs/polyalgos.md#function-vertex_enumeration_auctus): Efficient vertex enumeration algorithm for a problem of a form:

---

## Code examples

```python
"""
A simple example program for 3d force polytope 
evaluation of a randomised 6dof robot 
"""
from pycapacity.robot import force_polytope_withfaces as polytope
import numpy as np

m = 3 # 3d forces
n = 6 # robot dof

J = np.array(np.random.rand(m,n)) # random jacobian matrix

t_max = np.ones(n)  # joint torque limits max and min
t_min = -np.ones(n)

vertices, faces = polytope(J,t_min, t_max) # calculate the polytope vertices and faces

print(vertices) # display the vertices
```


```python
"""
A simple example program 3d force polytope 
evaluation of a randomised 30 muscle 7dof 
human musculoskeletal model 
"""
from pycapacity.human import force_polytope as polytope
import numpy as np

L = 30 # muscles
m = 3 # 3d forces
n = 6 # robot dof

J = np.array(np.random.rand(m,n))*2-1 # random jacobian matrix
N = np.array(np.random.rand(n,L))*2-1 # random moment arm matrix

F_max = 1000*np.ones(L)  # muscle forces limits max and min
F_min = np.zeros(L)

vertices, H,d, faces = polytope(J,N, F_min, F_max, 0.2) # calculate the polytope vertices and faces

print(vertices) # display the vertices
```

See [`demo_notebook.ipynb`](https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/demo_notebook.ipynb) for one example use case of the module.
