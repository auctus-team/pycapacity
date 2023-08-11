About pycapacity
################

An efficient task-space capacity calculation package for robotics and biomechanics


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

Implemented metrics
------------------------------------

This package implements polytope and ellipsoid based physical ability metrics for robotic manipulators and human musculoskeletal models.


Robotic manipulator capacity metrics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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


Performance evaluation of polytope metrics
---------------------------------------------

The applicable methods to evaluate different polytope based metrics depend on the family of problems they correspond to. 
Therefore this section brings the information about which algorithm is used for which polytope 
metric and provides a brief performance evaluation their execution times.

.. list-table:: Polytope algorithms used for different robot polytope metrics and their performance evaluation
    :widths: 25 25 50 50
    :header-rows: 1

    * - Polytope Metric
      - Algorithm
      - Problem type
      - Execution time [ms]  mean  :math:`\pm` std. (max)
    * - Velocity 
      - HPSM 
      - :math:`x=By,~ y \in [y_{min}, y_{max}]`
      - 3.6 :math:`\pm` 0.21 (5.7)
    * - Acceleration 
      -  HPSM 
      - :math:`x=By,~ y \in [y_{min}, y_{max}]`
      - 6.6 :math:`\pm` 1.4 (14.2)
    * - Force  
      - VEPOLI$^2$ 
      - :math:`Ax=b, ~ b \in [b_{min}, b_{max}]`
      - 6.8 :math:`\pm` 0.88 (16.4)
    * - Force intersection 
      -  VEPOLI$^2$ 
      - :math:`Ax=b,~ b \in [b_{min}, b_{max}]`
      - 98.2 :math:`\pm` 29.33 (165.8)
    * - Force sum 
      -  VEPOLI$^2$ 
      - :math:`Ax=b,~ b \in [b_{min}, b_{max}]` 
      - 17.1 :math:`\pm` 3.4 (44.9)
    * - Reachable space 
      -  ICHM 
      - :math:`x=By,~  y \in P_{y}`
      - 30.5 :math:`\pm` 6.6 (76.7)

The average execution time is calculated using 7 dof Franka Emika panda robot, the model was used with ``pinocchio`` software. 
All the experiments are run on a computer equipped with 1.90GHz Intel i7-8650U processor. The results are obtained using 
the benchmarking script provided in the ``examples`` folder, `script link <https://github.com/auctus-team/pycapacity/blob/master/examples/scripts/benchmarking/polytope_robot_performance_analysis_pinocchio.py>`_.


In case of human musculoskeletal models the methods used are given in the table below.


.. list-table:: Polytope algorithms used for different human polytope metrics and their performance evaluation
    :widths: 25 25 50 50
    :header-rows: 1

    * - Polytope Metric
      - Algorithm
      - Problem type
      - Execution time [ms]  mean :math:`\pm` std. (max)
    * - Force  
      - ICHM 
      - :math:`Ax=By,~ y \in [y_{min}, y_{max}]` 
      - 186.8 :math:`\pm` 45.6 (281.6)
    * - Acceleration 
      -  HPSM or ICHM 
      - :math:`x=By,~ y \in [y_{min}, y_{max}]` 
      -  378.8 :math:`\pm` 62.3 (643.7)
    * - Velocity 
      - ICHM 
      - :math:`x=By,~ y \in P_{y}` 
      - 223.1 :math:`\pm` 60.4 (389.1)

The average execution time was calculated using 50 muscle 7 dof musculoskeletal model introduced by Holzbaur, the model was used with ``biorbd`` biomechanics software. 
The experiments are run on a computer equipped with 1.90GHz Intel i7-8650U processor. The results are obtained using the benchmarking script 
provided in the `examples` folder, `script link <https://github.com/auctus-team/pycapacity/blob/master/examples/scripts/benchmarking/polytope_human_performance_analysis_biorbd.py>`_.

As these times can vary significantly depending on the complexity of the model used and the hardware it is run on, 
the users are encouraged to run the benchmark scripts themselves to get the most accurate results. 
This package provides several benchmarking scripts in the ``examples`` folder, see link for more 
details: `link <https://github.com/auctus-team/pycapacity/tree/master/examples/scripts/benchmarking>`_.



Compatible libraries
---------------------------------------------

The package is compatible with the following libraries:

.. list-table::
    :widths: 25 30
    :header-rows: 1

    * - Library
      - Example
    * - `OpenSim <https://github.com/opensim-org/opensim-core>`_
      - `Tutorial <examples/opensim.html>`_
    * - `pyomeca biorbd <https://github.com/pyomeca/biorbd>`_
      - `Tutorial <examples/pyomeca.html>`_
    * - `pinocchio <https://github.com/stack-of-tasks/pinocchio>`_
      - `Tutorial <examples/pinocchio.html>`_
    * - `Robotics toolbox <https://github.com/petercorke/robotics-toolbox-python>`_
      - `Tutorial <examples/robotics_toolbox.html>`_
    * - `Robot Operating System (ROS) <https://ros.org/>`_
      - `Tutorial <examples/ROS.html>`_





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
   
