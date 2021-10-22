
## Python real-time capable robot capacity module

![](https://gitlab.inria.fr/askuric/polytope_vertex_search/-/raw/master/images/rviz_screenshot_2020.png)

In this directory you can find the generic robot capacity calculation module called `pycapacity` which you can easily integrate in your python project, for example
```python
import pycapacity.pycapacity as capacity
```

This module integrates several velocity and force capacity calculation functions based on manipulability ellipsoids and polytopes. All the polytope functions have been implemented according to the paper:

[**On-line force capability evaluation based on efficient polytope vertex search**](https://arxiv.org/abs/2011.05226)<br> 
by Antun Skuric, Vincent Padois and David Daney<br> Published on ICRA2021


## Module functions

- [`acceleration_ellipsoid`](./docs/pycapacity.md#function-acceleration_ellipsoid): acceleration ellipsoid calculation (dynamic manipulability ellipsoid)
- [`acceleration_polytope`](./docs/pycapacity.md#function-acceleration_polytope): Acceleration polytope calculating function
- [`acceleration_polytope_withfaces`](./docs/pycapacity.md#function-acceleration_polytope_withfaces): Acceleration polytope calculating function
- [`force_ellipsoid`](./docs/pycapacity.md#function-force_ellipsoid): force manipulability ellipsoid calculation
- [`force_polytope`](./docs/pycapacity.md#function-force_polytope): Force polytope representing the capacities of the two robots in a certain configuration
- [`force_polytope_intersection`](./docs/pycapacity.md#function-force_polytope_intersection): Force polytope representing the intersection of the capacities of the two robots in certain configurations.
- [`force_polytope_intersection_withfaces`](./docs/pycapacity.md#function-force_polytope_intersection_withfaces): Force polytope representing the intersection of the capacities of the two robots in certain configurations.
- [`force_polytope_sum_withfaces`](./docs/pycapacity.md#function-force_polytope_sum_withfaces): Force polytope representing the minkowski sum of the capacities of the two robots in certain configurations.
- [`force_polytope_withfaces`](./docs/pycapacity.md#function-force_polytope_withfaces): Force polytope representing the capacities of the two robots in a certain configuration.
- [`hyper_plane_shift_method`](./docs/pycapacity.md#function-hyper_plane_shift_method): Hyper plane shifting method implementation used to solve problems of a form:
- [`make_2d`](./docs/pycapacity.md#function-make_2d): Take a list of 3D(cooplanar) points and make it 2D
- [`make_unique`](./docs/pycapacity.md#function-make_unique): Remove repetitions of columns
- [`order_index`](./docs/pycapacity.md#function-order_index): Order clockwise 2D points
- [`velocity_ellipsoid`](./docs/pycapacity.md#function-velocity_ellipsoid): velocity manipulability ellipsoid calculation
- [`velocity_polytope`](./docs/pycapacity.md#function-velocity_polytope): Velocity polytope calculating function
- [`velocity_polytope_withfaces`](./docs/pycapacity.md#function-velocity_polytope_withfaces): Velocity polytope calculating function, with faces


Algorithms:
- [`hyper_plane_shift_method`](./docs/pycapacity.md#function-hyper_plane_shift_method): Hyper plane shifting method implementation used to solve problems of a form:
---

## Code examples
See [`demo_notebook.ipynb`](./demo_notebook.ipynb) for one example use case of the module.
