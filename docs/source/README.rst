About pycapacity
################

Real-time capable task-space capacity calculation python pip package


.. Note::

  | 📢 New version of the ``pycapacity`` package is out- version v2.0! 
  | 🚨 Beware because there are some breaking changes in the API,  please check the `changelog <changelog.html>`_ for more details.


|
.. image:: https://img.shields.io/pypi/v/pycapacity
    :alt: Alternative text

.. image:: https://github.com/auctus-team/pycapacity/actions/workflows/python-app.yml/badge.svg
  :alt: Alternative text

.. image:: https://img.shields.io/pypi/dm/pycapacity?color=blue&label=pip%20downloads 
  :alt: Alternative text

.. image:: https://github.com/auctus-team/pycapacity/actions/workflows/main.yml/badge.svg
  :alt: Alternative text


|
.. image:: https://raw.githubusercontent.com/auctus-team/pycapacity/master/images/comparison.gif
  :width: 200
  :alt: Alternative text

.. image:: https://github.com/auctus-team/pycapacity/blob/master/images/human_poly.gif?raw=true
  :width: 200
  :alt: Alternative text


.. image:: images/output2.gif
  :height: 200
  :alt: Alternative text

The ``pycapacity`` package provides a framework for the generic task-space capacity calculation for:

* Robotic serial manipulators - ``pycapacity.robot``
* Human musculoskeletal models - ``pycapacity.human```

This package also provides a module ``pycapacity.algorithms`` with a set of polytope evaluation algorithms for standard polytope formulations, that can be used as a standalone library.

Additionally, ``pycapacity.visual`` module provides a set of visualisaiton tools using the ``matplotlib`` for visualising 2d and 3d polytopes.

Robotic manipulator capacity metrics
------------------------------------

.. image:: https://raw.githubusercontent.com/auctus-team/pycapacity/master/images/robot.png
  :height: 250
  :alt: Alternative text

For the robotic manipulators the package integrates several velocity, force and acceleration capacity calculation functions based on ellipsoids:
- Velocity (manipulability) ellipsoid  

    .. math:: E_{v} = \{\dot{x} ~| \dot{x} = J\dot{q},~ ||\dot{q}||\leq1 \}
- Acceleration (dynamic manipulability) ellipsoid  

    .. math:: E_{a} = \{\ddot{x} ~| \ddot{x} = JM^{-1}\tau,~ ||\tau||\leq1 \}
- Force ellipsoid 

    .. math:: E_{f} = \{{f} ~| J^{T}f = \tau,~ ||\tau||\leq1 \}

And polytopes: 
- Velocity polytope  

    .. math:: P_{v} = \{\dot{x} ~| \dot{x} = J\dot{q},~ \dot{q}_{min}\leq\dot{q}\leq\dot{q}_{max} \}
- Acceleration polytope 

    .. math:: P_{a} = \{\ddot{x} ~| \ddot{x} = JM^{-1}\tau,~ \tau_{min}\leq\tau\leq\tau_{max} \}
- Force polytope  

    .. math:: P_{f} = \{f ~| J^{T}f = \tau,~ \tau_{min}\leq\tau\leq\tau_{max} \}
- Force polytopes *Minkowski sum and intersection*  

    .. math::  P_{\cap} = \mathcal{P}_{f1} \cap \mathcal{P}_{f1} \qquad P_{\oplus} = \mathcal{P}_{f1} \oplus \mathcal{P}_{f1} 

Where :math:`J` is the robot jacobian matrix, :math:`f` is the vector of cartesian forces, :math:`\dot{x}` and :math:`\ddot{x}` are vectors fo cartesian velocities and accretions, :math:`\dot{q}` is the vector of the joint velocities and :math:`\tau` is the vector of joint torques.

- New 📢 : Reachable space approximation of in the desired horizon of interest :math:`\Delta t_{h}` using the convex polytope formulation:

    .. math::  P_x = \{\Delta x~ |~ \Delta{x} = JM^{-1}\tau \Delta t_{h}^2/2,
    .. math:: {\tau}_{min} \leq \tau \leq {\tau}_{max},
    .. math::  \dot{q}_{min} \leq M^{-1}\tau \Delta t_{h}  \leq \dot{q}_{max},
    .. math::  {q}_{min} \leq M^{-1}\tau \Delta t_{h}^2/2  \leq {q}_{max} \}

  | Based on the paper:
  | `Approximating robot reachable space using convex polytopes. <https://arxiv.org/pdf/2211.17054.pdf>`_
  | by Skuric, Antun, Vincent Padois, and David Daney. 
  | In: Human-Friendly Robotics 2022: HFR: 15th International Workshop on Human-Friendly Robotics. Cham: Springer International Publishing, 2023.

  See the full formulaiton in the `api docs <pycapacity.robot.html#pycapacity.robot.reachable_space_approximation>`_.

Human musculoskeletal models capacity metrics
---------------------------------------------

.. image:: https://raw.githubusercontent.com/auctus-team/pycapacity/master/images/force.png
  :height: 250
  :alt: Alternative text

For the human musculoskeletal models this package implements the polytope metrics:
- Velocity polytope  

    .. math:: P_{v} = \{\dot{x} ~| \dot{l} = L\dot{q},~ \dot{x} = J\dot{q},~ \dot{l}_{min}\leq\dot{l}\leq\dot{l}_{max} \}
- Acceleration polytope   

    .. math:: P_{a} = \{\ddot{x} ~| \ddot{x} = JM^{-1}NF,~ F_{min}\leq F\leq F_{max} \}
- Force polytope   

    .. math:: P_{f} = \{f ~| J^Tf = NF,~ F_{min}\leq F\leq F_{max} \}

Where :math:`J` is the model's jacobian matrix, :math:`L` si the muscle length jacobian matrix, :math:`N= -L^T` is the moment arm matrix, :math:`f` is the vector of cartesian forces, :math:`$\dot{x}` and :math:`\ddot{x}` are vectors fo cartesian velocities and accretions, :math:`\dot{q}` is the vector of the joint velocities, :math:`\tau` is the vector of joint torques, :math:`\dot{l}` is the vector of the muscle stretching velocities and :math:`F` is the vector of muscular forces. 

Polytope evaluation algorithms
------------------------------
There are three methods implemented in this paper to resolve all the polytope calculations:

* Hyper-plane shifting method (HPSM)
* Iterative convex hull method (ICHM)
* Vertex enumeration (VEPOLI\ :sup:`2`)

All of the methods are implemented in the module `pycapacity.algorithms` and can be used as standalone functions.  See in [docs for more info](pycapacity.algorithms.html). 

Hyper-plane shifting method (HPSM)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
| Based on the paper:
| `Characterization of Parallel Manipulator Available Wrench Set Facets <http://www.lirmm.fr/krut/pdf/2010_gouttefarde_ark-0602650368/2010_gouttefarde_ark.pdf>`_
| by Gouttefarde M., Krut S. 
| In: Lenarcic J., Stanisic M. Advances in Robot Kinematics: Motion in Man and Machine. Springer, Dordrecht (2010)

This method finds the half-space representation of the polytope of a class:

.. math:: P = \{ x ~|~ x = By, \quad y_{min}\leq y \leq y_{max} \}

To find the vertices of the polytope after finding the half-space representation :math:`Hx \leq d` an convex-hull algorithm is used. 

The method is a part of the ``pycapacity.algorithms``` module ``hyper_plane_shift_method``, See in `docs for more info <pycapacity.algorithms.html#pycapacity.algorithms.hyper_plane_shift_method>`_ 

Iterative convex-hull method (ICHM)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| Based on the paper:
| `On-line feasible wrench polytope evaluation based on human musculoskeletal models: an iterative convex hull method <https://hal.inria.fr/hal-03369576>`_ 
| by A.Skuric, V.Padois, N.Rezzoug and D.Daney
| Published in RAL & ICRA2022 

This method finds both vertex and half-space representation of the class of polytopes:


.. math:: P = \{ x ~|~ Ax = By, \quad y_{min}\leq y \leq y_{max} \}


And it can be additionally extended to the case where there is an additional projection matrix $P$ making a class of problems:

.. math:: P = \{ x ~|~ x= Pz, Az = By, \quad y_{min}\leq y \leq y_{max} \}


The method is a part of the ``pycapacity.algorithms`` module ``iterative_convex_hull_method``. See the `docs for more info <pycapacity.algorithms.html#pycapacity.algorithms.iterative_convex_hull_method>`_

Vertex enumeration algorithm (VEPOLI\ :sup:`2`)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| Based on the paper:
| `On-line force capability evaluation based on efficient polytope vertex search <https://arxiv.org/abs/2011.05226>`_ 
| by A.Skuric, V.Padois and D.Daney
| Published on ICRA2021

This method finds vertex representation of the class of polytopes:

.. math:: P = \{ x ~|~ Ax = y, \quad y_{min}\leq y \leq y_{max} \}


To find the half-space representation (faces) of the polytope after finding the vertex representation  an convex-hull algorithm is used. 

The method is a part of the ``pycapacity.algorithms`` module ``vertex_enumeration_vepoli2``. See the `docs for more info <pycapacity.algorithms.html#pycapacity.algorithms.vertex_enumeration_vepoli2>`_

Read more
---------

.. toctree::
   :maxdepth: 1

   🚀 INSTALLATION <install.md>
   📄 API DOCS <pycapacity>
   📝 CONTRIBUTING & ISSUES <contirbuting.md>
   🎓 TUTORIALS <examples/index>
   
