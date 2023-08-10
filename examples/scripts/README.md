# Python Example Scripts

The Python scripts are located in the `examples/scripts` folder.
Currently, the following scripts are available where no dependencies are required:
- **Four link robot**
    - [Acceleration Polytope and Ellipsoid](4dof_robot_examples/acceleration_capacity.py)
    - [Force Polytope and Ellipsoid](4dof_robot_examples/velocity_capacity.py)
    - [Velocity Polytope and Ellipsoid](4dof_robot_examples/velocity_capacity.py)
- Random robot model
    - [Polytope](robot_random_model.py)
    - [Comparison Polytope and Ellipsoid](robot_random_polytope_ellispoid.py)
- [Random human model](human_random_model.py)

The other scripts require additional dependencies:
- [pinocchio](pinocchio.py)
- [robotics toolbox with swift](robotics_toolbox_swift.py)
- [robotics toolbox with pyplot](robotics_toolbox_pyplot.py)
- [pyomeca](pyomeca.py)

## Running the example scripts

Open the terminal in this folder `examples/scripts` and if all the dependencies are met you should be able to run the scripts with
```
python name_of_the_script.py
```