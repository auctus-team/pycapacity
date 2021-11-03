<!-- markdownlint-disable -->

# API Overview

## Modules

- [`algorithms`](./algorithms.md#module-algorithms)
- [`human`](./human.md#module-human)
- [`robot`](./robot.md#module-robot)
- [`visual`](./visual.md#module-visual)

## Classes

- No classes

## Functions

- [`algorithms.face_index_to_vertex`](./algorithms.md#function-face_index_to_vertex): Helping function for transforming the list of faces with indexes to the vertices
- [`algorithms.hyper_plane_shift_method`](./algorithms.md#function-hyper_plane_shift_method): Hyper plane shifting method implementation used to solve problems of a form:
- [`algorithms.iterative_convex_hull_method`](./algorithms.md#function-iterative_convex_hull_method): A function calculating the polytopes of achievable x for equations form:
- [`algorithms.order_index`](./algorithms.md#function-order_index): Order clockwise 2D points
- [`algorithms.stack`](./algorithms.md#function-stack)
- [`algorithms.vertex_enumeration_auctus`](./algorithms.md#function-vertex_enumeration_auctus): Efficient vertex enumeration algorithm for a problem of a form:
- [`human.acceleration_polytope`](./human.md#function-acceleration_polytope): A function calculating the polytopes of achievable accelerations
- [`human.force_polytope`](./human.md#function-force_polytope): A function calculating the polytopes of achievable foreces based 
- [`human.joint_torques_polytope`](./human.md#function-joint_torques_polytope): A function calculating the polytopes of achievable joint torques
- [`human.torque_to_muscle_force`](./human.md#function-torque_to_muscle_force): A function calculating muscle forces needed to create the joint torques tau
- [`human.velocity_polytope`](./human.md#function-velocity_polytope): A function calculating the polytopes of achievable velocity based 
- [`robot.acceleration_ellipsoid`](./robot.md#function-acceleration_ellipsoid): acceleration ellipsoid calculation (dynamic manipulability ellipsoid)
- [`robot.acceleration_polytope`](./robot.md#function-acceleration_polytope): Acceleration polytope calculating function
- [`robot.acceleration_polytope_withfaces`](./robot.md#function-acceleration_polytope_withfaces): Acceleration polytope calculating function
- [`robot.force_ellipsoid`](./robot.md#function-force_ellipsoid): force manipulability ellipsoid calculation
- [`robot.force_polytope`](./robot.md#function-force_polytope): Force polytope representing the capacities of the two robots in a certain configuration
- [`robot.force_polytope_intersection`](./robot.md#function-force_polytope_intersection): Force polytope representing the intersection of the capacities of the two robots in certain configurations.
- [`robot.force_polytope_intersection_withfaces`](./robot.md#function-force_polytope_intersection_withfaces): Force polytope representing the intersection of the capacities of the two robots in certain configurations.
- [`robot.force_polytope_sum_withfaces`](./robot.md#function-force_polytope_sum_withfaces): Force polytope representing the minkowski sum of the capacities of the two robots in certain configurations.
- [`robot.force_polytope_withfaces`](./robot.md#function-force_polytope_withfaces): Force polytope representing the capacities of the two robots in a certain configuration.
- [`robot.velocity_ellipsoid`](./robot.md#function-velocity_ellipsoid): velocity manipulability ellipsoid calculation
- [`robot.velocity_polytope`](./robot.md#function-velocity_polytope): Velocity polytope calculating function
- [`robot.velocity_polytope_withfaces`](./robot.md#function-velocity_polytope_withfaces): Velocity polytope calculating function, with faces
- [`visual.plot_polytope_faces`](./visual.md#function-plot_polytope_faces): Polytope faces plotting function in 2d and 3d
- [`visual.plot_polytope_vertex`](./visual.md#function-plot_polytope_vertex): Polytope vertices plotting function in 2d and 3d


---

_This file was automatically generated via [lazydocs](https://github.com/ml-tooling/lazydocs)._
