An efficient task-space capacity calculation package for robotics and biomechanics
##############################################################################################

.. Note::

  | 📢 New version of the ``pycapacity`` package is out- version v2.0.1! 

|
.. image:: https://img.shields.io/pypi/v/pycapacity

.. image:: https://github.com/auctus-team/pycapacity/actions/workflows/python-app.yml/badge.svg

.. image:: https://img.shields.io/pypi/dm/pycapacity?color=blue&label=pip%20downloads

.. image:: https://github.com/auctus-team/pycapacity/actions/workflows/main.yml/badge.svg

|
.. image:: https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/raw/master/images/comparison.gif
  :height: 200
  :alt: Alternative text

.. image:: https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/raw/master/images/human_poly.gif
  :height: 200
  :alt: Alternative text

.. image:: images/output2.gif
  :height: 200
  :alt: Alternative text

What is ``pycapacity``?
------------------------------------

Python package ``pycapacity`` provides a set of tools for evaluating task space physical ability metrics for humans and robots, based on polytopes and ellipsoids. 
The aim of ``pycapacity`` is to provide a set of efficient tools for their evaluation in an easy to use framework that can be easily integrated with standard robotics 
and biomechanics libraries. The package implements several state of the art algorithms for polytope evaluation that bring many of the 
polytope metrics to the few milliseconds evaluation time, making it possible to use them in online and interactive applications. 

The package can be easily interfaced with standard libraries for robotic manipulator rigid body simulation such as ``robotic-toolbox`` 
or ``pinocchio``, as well as human musculoskeletal model biomechanics 
softwares ``opensim`` and ``biorbd``. The package can also be used with the Robot Operating System (``ROS``).

The package additionally implements a set of visualization tools for polytopes and ellipsoids based on the
Python package ``matplotlib`` intended for fast prototyping and quick and interactive visualization.


Learn more about the ``pycapacity`` package
-------------------------------------------
.. toctree::
   :maxdepth: 1

    About <README.md>
    🌟 Implemented Algorithms <algorithms.rst>
    🚀 Installation <install.md>
    📄 API Docs <pycapacity>
    📝 Contributing & Issues <contirbuting.md>
    🎓 Tutorials <examples/index>
    📑 Changelog <changelog>
   
   