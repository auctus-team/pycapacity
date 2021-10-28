<!-- markdownlint-disable -->

# API Overview

## Modules

- [`human`](./human.md#module-human)
- [`polyalgos`](./polyalgos.md#module-polyalgos)
- [`robot`](./robot.md#module-robot)

## Classes

- No classes

## Functions

- [`human.acceleration_polytope`](./human.md#function-acceleration_polytope): A function calculating the polytopes of achievable accelerations
- [`human.force_polytope`](./human.md#function-force_polytope): A function calculating the polytopes of achievable foreces based 
- [`human.joint_torques_polytope`](./human.md#function-joint_torques_polytope): A function calculating the polytopes of achievable joint torques
- [`human.torque_to_muscle_force`](./human.md#function-torque_to_muscle_force): A function calculating muscle forces needed to create the joint torques tau
- [`human.velocity_polytope`](./human.md#function-velocity_polytope): A function calculating the polytopes of achievable velocity based 
- [`polyalgos.hyper_plane_shift_method`](./polyalgos.md#function-hyper_plane_shift_method): Hyper plane shifting method implementation used to solve problems of a form:
- [`polyalgos.iterative_convex_hull_method`](./polyalgos.md#function-iterative_convex_hull_method): A function calculating the polytopes of achievable x for equations form:
- [`polyalgos.make_2d`](./polyalgos.md#function-make_2d): Take a list of 3D(cooplanar) points and make it 2D
- [`polyalgos.make_unique`](./polyalgos.md#function-make_unique): Remove repetitions of columns
- [`polyalgos.order_index`](./polyalgos.md#function-order_index): Order clockwise 2D points
- [`polyalgos.stack`](./polyalgos.md#function-stack)
- [`polyalgos.vertex_enumeration_auctus`](./polyalgos.md#function-vertex_enumeration_auctus): Efficient vertex enumeration algorithm for a problem of a form:
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


---

_This file was automatically generated via [lazydocs](https://github.com/ml-tooling/lazydocs)._
