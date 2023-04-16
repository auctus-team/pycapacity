# Installation

All you need to do to install it is:
```
pip install pycapacity
```
And include it to your python project
```python
import pycapacity.robot 
# and/or
import pycapacity.human 
#and/or
import pycapacity.algorithms 
#and/or
import pycapacity.visual 
```

Other way to install the code is by installing it directly from the git repo:
```
pip install git+https://github.com/auctus-team/pycapacity.git
```

### Installing from source

In order to install the package from source clone the git repository 
```sh
git clone https://github.com/auctus-team/pycapacity.git
```
and install it using 
```sh
cd pycapacity
pip install -e .
```
then your can make import the `pycapacity` package as usual in your applications.

Furthermore if installing from source you can run all the unit tests in order to verify that the installation went well
```sh
pytest tests/*
```