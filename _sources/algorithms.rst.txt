
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


Performance evaluation of polytope metrics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
      - VEPOLI :math:`\!^2` 
      - :math:`Ax=b, ~ b \in [b_{min}, b_{max}]`
      - 6.8 :math:`\pm` 0.88 (16.4)
    * - Force intersection 
      - VEPOLI :math:`\!^2` 
      - :math:`Ax=b,~ b \in [b_{min}, b_{max}]`
      - 98.2 :math:`\pm` 29.33 (165.8)
    * - Force sum 
      - VEPOLI :math:`\!^2` 
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

