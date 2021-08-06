
## Python capacity module

In this directory you can find the generic robot capacity calculation module called `robot_capacity_solver` which you can easily integrate in your python project, for example
```python
import pycapacity.pycapacity as capacity
```

# API Overview

## Modules

- [`pycapacity`](./docs/pycapacity.md#module-pycapacity)

## Functions

- [`pycapacity.force_polytope`](./docs/pycapacity.md#function-force_polytope): Force polytope representing the capacities of the two robots in a certain configuration
- [`pycapacity.force_polytope_intersection`](./docs/pycapacity.md#function-force_polytope_intersection): Force polytope representing the intersection of the capacities of the two robots in certain configurations.
- [`pycapacity.force_polytope_intersection_withfaces`](./docs/pycapacity.md#function-force_polytope_intersection_withfaces): Force polytope representing the intersection of the capacities of the two robots in certain configurations.
- [`pycapacity.force_polytope_sum_withfaces`](./docs/pycapacity.md#function-force_polytope_sum_withfaces): Force polytope representing the minkowski sum of the capacities of the two robots in certain configurations.
- [`pycapacity.force_polytope_withfaces`](./docs/pycapacity.md#function-force_polytope_withfaces): Force polytope representing the capacities of the two robots in a certain configuration.
- [`pycapacity.make_2d`](./docs/pycapacity.md#function-make_2d): Take a list of 3D(cooplanar) points and make it 2D
- [`pycapacity.make_unique`](./docs/pycapacity.md#function-make_unique): Remove repetitions of columns
- [`pycapacity.manipulability_force`](./docs/pycapacity.md#function-manipulability_force): force manipulability calculation
- [`pycapacity.manipulability_velocity`](./docs/pycapacity.md#function-manipulability_velocity): velocity manipulability calculation
- [`pycapacity.order_index`](./docs/pycapacity.md#function-order_index): Order clockwise 2D points

---


See `demo_notebook.ipynb` for one example use case of the module.
