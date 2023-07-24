Real-time capable task-space capacity calculation python module
###############################################################

.. Note::

  | 📢 New version of the ``pycapacity`` package is out- version v2.0! 
  | 🚨 Beware beacause there are some breaking changes in the API,  please check the `changelog <changelog.html>`_ for more details.

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


The ``pycapacity`` package provides a framework for the generic task-space capacity calculation for:

 *  Robotic serial manipulators - ``pycapacity.robot``
 *  Human musculoskeletal models - ``pycapacity.human``

This package also provides a module ``pycapacity.algorithms`` with a set of polytope evaluation algorithms for standard polytope formulations, that can be used as a standalone library.

Additionally, ``pycapacity.visual`` module provides a set of visualisaiton tools using the ``matplotlib`` for visualising 2d and 3d polytopes.

Learn more about the ``pycapacity`` package
-------------------------------------------
.. toctree::
   :maxdepth: 1

   About <README.md>
   🚀 Installation <install.md>
   📄 API Docs <pycapacity>
   📝 Contributing & Issues <contirbuting.md>
   🎓 Tutorials <examples/index>
   📑 Changelog <changelog>
   