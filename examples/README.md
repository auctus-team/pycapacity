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


### Running the example notebooks

Open the terminal in this folder `examples/notebooks` and if you do not have jupyter installed, do install it

```
pip install jupyter
```
or with anaconda
```
conda install -c anaconda-forge jupyter
```
Once you have jupyter installed just run and you'll be able to navigate the example scripts and run them
```
jupyter lab
```

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
- [pinocchio](scripts/pinocchio_example.py)
- [robotics toolbox with swift](scripts/robotics_toolbox_swift.py)
- [robotics toolbox with pyplot](scripts/robotics_toolbox_pyplot.py)
- [pyomeca](scripts/pyomeca_example.py)
- mujoco examples
    - [Froce polytope for different robots](scripts/mujoco_example.py)
    - [Panda robot reachable workspace](scripts/mujoco_polytope_ellispoid.py)


Performance testing and benchmarking scripts:
- [Performance testing script for polytope algorithms using radom robot models](scripts/benchmarking/polytope_robot_performance_analysis.py)
- [Performance testing script for polytope algorithms using radom human models](scripts/benchmarking/polytope_human_performance_analysis.py)
- [Performance testing script for polytope algorithms using pinocchio](scripts/benchmarking/polytope_robot_performance_analysis_pinocchio.py)
- [Performance testing script for polytope algorithms using biorbd](scripts/benchmarking/polytope_human_performance_analysis_biorbd.py)


### Running the example scripts

Open the terminal in this folder `examples/scripts` and if all the dependencies are met you should be able to run the scripts with
```
python name_of_the_script.py
```