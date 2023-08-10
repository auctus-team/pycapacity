# Examples folder

Two types of examples are provided:
- Jupyter notebooks
- Python scripts

## Jupyter notebooks

The Jupyter notebooks are located in the `examples/notebooks` folder.
Currently, the following notebooks are available where no dependencies are required:
- [Four link robot](notebooks/four_link.ipynb)
- [Randomised robot and human model](notebooks/demo_simple.ipynb)


The other notebooks require additional dependencies:
- [pinocchio](notebooks/pinocchio.ipynb)
- [robotics toolbox with swift](notebooks/robotics_toolbox_swift.ipynb)
- [robotics toolbox with pyplot](notebooks/robotics_toolbox_pyplot.ipynb)

## Python scripts

The Python scripts are located in the `examples/scripts` folder.
Currently, the following scripts are available where no dependencies are required:
- **Four link robot**
    - [Acceleration Polytope and Ellipsoid](scripts/4dof_robot_examples/acceleration_capacity.py)
    - [Force Polytope and Ellipsoid](scripts/4dof_robot_examples/velocity_capacity.py)
    - [Velocity Polytope and Ellipsoid](scripts/4dof_robot_examples/velocity_capacity.py)
- Random robot model
    - [Polytope](scripts/robot_random_model.py)
    - [Comparison Polytope and Ellipsoid](scripts/robot_random_polytope_ellispoid.py)
- [Random human model](scripts/human_random_model.py)

The other scripts require additional dependencies:
- [pinocchio](scripts/pinocchio.py)
- [robotics toolbox with swift](scripts/robotics_toolbox_swift.py)
- [robotics toolbox with pyplot](scripts/robotics_toolbox_pyplot.py)
- [pyomeca](scripts/pyomeca.py)