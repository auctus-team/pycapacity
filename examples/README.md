# Examples folder

Two types of examples are provided:
- Jupyter notebooks
- Python scripts

## Jupyter notebooks

The Jupyter notebooks are located in the `examples/notebooks` folder.
Currently, the following notebooks are available where no dependencies are required:
- [Four link robot](examples/notebooks/four_link.ipynb)
- [Randomised robot and human model](examples/notebooks/demo_simple.ipynb)


The other notebooks require additional dependencies:
- [pinocchio](examples/notebooks/pinocchio.ipynb)
- [robotics toolbox with swift](examples/notebooks/robotics_toolbox_swift.ipynb)
- [robotics toolbox with pyplot](examples/notebooks/robotics_toolbox_pyplot.ipynb)

## Python scripts

The Python scripts are located in the `examples/scripts` folder.
Currently, the following scripts are available where no dependencies are required:
- **Four link robot**
    - [Acceleration Polytope and Ellipsoid](examples/scripts/4dof_robot_examples/acceleration_capacity.py)
    - [Force Polytope and Ellipsoid](examples/scripts/4dof_robot_examples/velocity_capacity.py)
    - [Velocity Polytope and Ellipsoid](examples/scripts/4dof_robot_examples/velocity_capacity.py)
- Random robot model
    - [Polytope](examples/scripts/robot_random_model.py)
    - [Comparison Polytope and Ellipsoid](examples/scripts/robot_random_polytope_ellispoid.py)
- [Random human model](examples/scripts/human_random_model.py)

The other scripts require additional dependencies:
- [pinocchio](examples/scripts/pinocchio.py)
- [robotics toolbox with swift](examples/scripts/robotics_toolbox_swift.py)
- [robotics toolbox with pyplot](examples/scripts/robotics_toolbox_pyplot.py)
- [pyomeca](examples/scripts/pyomeca.py)