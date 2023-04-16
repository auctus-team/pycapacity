
# Real-time capable task-space capacity calculation python module


<img src="https://raw.githubusercontent.com/auctus-team/pycapacity/master/images/comparison.gif" style="height:250px"><img src="https://github.com/auctus-team/pycapacity/blob/master/images/human_poly.gif?raw=true" style="height:250px">

The `pycapacity` package provides a framework for the generic task-space capacity calculation for:
- Robotic serial manipulators - `pycapacity.robot`
- Human musculoskeletal models - `pycapacity.human`

This package also provides a module `pycapacity.algorithms` with a set of polytope evaluation algorithms for standard polytope formulations, that can be used as a standalone library.

Additionally, `pycapacity.visual` module provides a set of visualisaiton tools using the `matplotlib` for visualising 2d and 3d polytopes.

## Robotic manipulator capacity metrics
<img src='https://raw.githubusercontent.com/auctus-team/pycapacity/master/images/robot.png' style="height:300px">

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
<img src='https://raw.githubusercontent.com/auctus-team/pycapacity/master/images/force.png' style="max-height:300px" height='200px'>

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

All of the methods are implemented in the module `pycapacity.algorithms` and can be used as standalone functions.  See in [docs for more info](pycapacity.algorithms.html). 

### Hyper-plane shifting method
[**Characterization of Parallel Manipulator Available Wrench Set Facets**](http://www.lirmm.fr/krut/pdf/2010_gouttefarde_ark-0602650368/2010_gouttefarde_ark.pdf)<br>
by Gouttefarde M., Krut S. <br>In: Lenarcic J., Stanisic M. (eds) Advances in Robot Kinematics: Motion in Man and Machine. Springer, Dordrecht (2010)

This method finds the half-space representation of the polytope of a class:
```
P = {x | x = By, y_min <= y <= y_max }
```
To find the vertices of the polytope after finding the half-space representation `Hx <= d` an convex-hull algorithm is used. 

The method is a part of the `pycapacity.algorithms` module `hyper_plane_shift_method`, See in [docs for more info](pycapacity.algorithms.html#pycapacity.algorithms.hyper_plane_shift_method). 

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

The method is a part of the `pycapacity.algorithms` module `iterative_convex_hull_method`. See the [docs for more info](pycapacity.algorithms.html#pycapacity.algorithms.iterative_convex_hull_method)

### Vertex enumeration auctus
[**On-line force capability evaluation based on efficient polytope vertex search**](https://arxiv.org/abs/2011.05226)<br> 
by A.Skuric, V.Padois and D.Daney<br> Published on ICRA2021

This method finds vertex representation of the class of polytopes:
```
P = {x | Ax = y, y_min <= y <= y_max }
``` 
To find the half-space representation (faces) of the polytope after finding the vertex representation  an convex-hull algorithm is used. 

The method is a part of the `pycapacity.algorithms` module `vertex_enumeration_auctus`. See the [docs for more info](pycapacity\.algorithms\.html#pycapacity\.algorithms\.vertex_enumeration_auctus)

## Package API docs

### Modules

- [`human`](pycapacity.human.html#module-pycapacity.human)
- [`robot`](pycapacity.robot.html#module-pycapacity.robot)
- [`algorithms`](pycapacity.algorithms.html#module-pycapacity.algorithms)
- [`visual`](pycapacity.visual.html#module-pycapacity.visual)

### Functions

Robot metrics
- [`robot.acceleration_ellipsoid`](pycapacity.robot.html#pycapacity.robot.acceleration_ellipsoid): acceleration ellipsoid calculation (dynamic manipulability ellipsoid)
- [`robot.acceleration_polytope`](pycapacity\.robot\.html#pycapacity\.robot\.acceleration_polytope): Acceleration polytope calculating function
- [`robot.acceleration_polytope_withfaces`](pycapacity\.robot\.html#pycapacity\.robot\.acceleration_polytope_withfaces): Acceleration polytope calculating function
- [`robot.force_ellipsoid`](pycapacity\.robot\.html#pycapacity\.robot\.force_ellipsoid): force manipulability ellipsoid calculation
- [`robot.force_polytope`](pycapacity\.robot\.html#pycapacity\.robot\.force_polytope): Force polytope representing the capacities of the two robots in a certain configuration
- [`robot.force_polytope_intersection`](pycapacity\.robot\.html#pycapacity\.robot\.force_polytope_intersection): Force polytope representing the intersection of the capacities of the two robots in certain configurations.
- [`robot.force_polytope_intersection_withfaces`](pycapacity\.robot\.html#pycapacity\.robot\.force_polytope_intersection_withfaces): Force polytope representing the intersection of the capacities of the two robots in certain configurations.
- [`robot.force_polytope_sum_withfaces`](pycapacity\.robot\.html#pycapacity\.robot\.force_polytope_sum_withfaces): Force polytope representing the minkowski sum of the capacities of the two robots in certain configurations.
- [`robot.force_polytope_withfaces`](pycapacity\.robot\.html#pycapacity\.robot\.force_polytope_withfaces): Force polytope representing the capacities of the two robots in a certain configuration.
- [`robot.velocity_ellipsoid`](pycapacity\.robot\.html#pycapacity\.robot\.velocity_ellipsoid): velocity manipulability ellipsoid calculation
- [`robot.velocity_polytope`](pycapacity\.robot\.html#pycapacity\.robot\.velocity_polytope): Velocity polytope calculating function
- [`robot.velocity_polytope_withfaces`](pycapacity\.robot\.html#pycapacity\.robot\.velocity_polytope_withfaces): Velocity polytope calculating function, with faces

Human metrics
- [`human.acceleration_polytope`](pycapacity\.human\.html#pycapacity\.human\.acceleration_polytope): A function calculating the polytopes of achievable accelerations
- [`human.force_polytope`](pycapacity\.human\.html#pycapacity\.human\.force_polytope): A function calculating the polytopes of achievable foreces based 
- [`human.joint_torques_polytope`](pycapacity\.human\.html#pycapacity\.human\.joint_torques_polytope): A function calculating the polytopes of achievable joint torques
- [`human.torque_to_muscle_force`](pycapacity\.human\.html#pycapacity\.human\.torque_to_muscle_force): A function calculating muscle forces needed to create the joint torques tau
- [`human.velocity_polytope`](pycapacity\.human\.html#pycapacity\.human\.velocity_polytope): A function calculating the  polytopes of achievable velocity based 


Algorithms
- [`algorithms.hyper_plane_shift_method`](pycapacity\.algorithms\.html#pycapacity\.algorithms\.hyper_plane_shift_method): Hyper plane shifting method implementation used to solve problems of a form:
- [`algorithms.iterative_convex_hull_method`](pycapacity\.algorithms\.html#pycapacity\.algorithms\.iterative_convex_hull_method): A function calculating the polytopes of achievable x for equations form:
- [`algorithms.vertex_enumeration_auctus`](pycapacity\.algorithms\.html#pycapacity\.algorithms\.vertex_enumeration_auctus): Efficient vertex enumeration algorithm for a problem of a form:


Visualisation tools
- [`visual.plot_polytope_faces`](pycapacity.visual.html#pycapacity.visual.plot_polytope_faces): Polytope faces plotting function in 2d and 3d
- [`visual.plot_polytope_vertex`](pycapacity.visual.html#fpycapacity.visual.plot_polytope_vertex): Polytope vertices plotting function in 2d and 3d
---