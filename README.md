
## Python robot capacity module

In this directory you can find the generic robot capacity calculation module called `pycapacity` which you can easily integrate in your python project, for example
```python
import pycapacity.pycapacity as capacity
```

## Functions
- [`force_polytope`](./docs/pycapacity.md#function-force_polytope): Force polytope representing the capacities of the two robots in a certain configuration
- [`force_polytope_intersection`](./docs/pycapacity.md#function-force_polytope_intersection): Force polytope representing the intersection of the capacities of the two robots in certain configurations.
- [`force_polytope_intersection_withfaces`](./docs/pycapacity.md#function-force_polytope_intersection_withfaces): Force polytope representing the intersection of the capacities of the two robots in certain configurations.
- [`force_polytope_sum_withfaces`](./docs/pycapacity.md#function-force_polytope_sum_withfaces): Force polytope representing the minkowski sum of the capacities of the two robots in certain configurations.
- [`force_polytope_withfaces`](./docs/pycapacity.md#function-force_polytope_withfaces): Force polytope representing the capacities of the two robots in a certain configuration.
- [`manipulability_force`](./docs/pycapacity.md#function-manipulability_force): force manipulability calculation
- [`manipulability_velocity`](./docs/pycapacity.md#function-manipulability_velocity): velocity manipulability calculation
---

## Code examples
See [`demo_notebook.ipynb`](.demo_notebook.ipynb) for one example use case of the module.
