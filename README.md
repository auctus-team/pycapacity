
## Python real-time capable robot capacity module

![](https://gitlab.inria.fr/askuric/polytope_vertex_search/-/raw/master/images/rviz_screenshot_2020.png)

In this directory you can find the generic robot capacity calculation module called `pycapacityRT` which you can easily integrate in your python project, for example
```python
import pycapacityRT.pycapacity as capacity
```

This module integrates several velocity and force capacity calculation functions based on manipulability ellipsoids and polytopes. All the polytope functions have been implemented according to the paper:

[**On-line force capability evaluation based on efficient polytope vertex search**](https://arxiv.org/abs/2011.05226)<br> 
by Antun Skuric, Vincent Padois and David Daney<br> Published on ICRA2021

## Module functions
- [`force_polytope`](./docs/pycapacity.md#function-force_polytope): Force polytope representing the capacities of the two robots in a certain configuration
- [`force_polytope_intersection`](./docs/pycapacity.md#function-force_polytope_intersection): Force polytope representing the intersection of the capacities of the two robots in certain configurations.
- [`force_polytope_intersection_withfaces`](./docs/pycapacity.md#function-force_polytope_intersection_withfaces): Force polytope representing the intersection of the capacities of the two robots in certain configurations.
- [`force_polytope_sum_withfaces`](./docs/pycapacity.md#function-force_polytope_sum_withfaces): Force polytope representing the minkowski sum of the capacities of the two robots in certain configurations.
- [`force_polytope_withfaces`](./docs/pycapacity.md#function-force_polytope_withfaces): Force polytope representing the capacities of the two robots in a certain configuration.
- [`manipulability_force`](./docs/pycapacity.md#function-manipulability_force): force manipulability calculation
- [`manipulability_velocity`](./docs/pycapacity.md#function-manipulability_velocity): velocity manipulability calculation
---

## Code examples
See [`demo_notebook.ipynb`](./demo_notebook.ipynb) for one example use case of the module.
