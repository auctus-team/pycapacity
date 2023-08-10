# Interactive Jupyter Notebooks

The Jupyter notebooks are located in the `examples/notebooks` folder.
Currently, the following notebooks are available where no dependencies are required:
- [Four link robot](four_link.ipynb)
- [Randomised robot and human model](demo_simple.ipynb)


The other notebooks require additional dependencies:
- [pinocchio](pinocchio.ipynb)
- [robotics toolbox with swift](robotics_toolbox_swift.ipynb)
- [robotics toolbox with pyplot](robotics_toolbox_pyplot.ipynb)


### Running the example notebooks

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