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

Read more
---------

.. toctree::
   :maxdepth: 1

      About <README.md>
      🌟 Implemented Algorithms <algorithms.rst>
      🚀 Installation <install.md>
      📄 API Docs <pycapacity>
      📝 Contributing & Issues <contirbuting.md>
      🎓 Tutorials <examples/index>
      📑 Changelog <changelog>
   
