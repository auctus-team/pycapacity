Changelog
=========


See `github releases <https://github.com/auctus-team/pycapacity/releases>`_ and `pypi <https://pypi.org/project/pycapacity/#history>`_ for more info. 

If wanting to use a specific version of the library, you can install it using pip, for example version ``1.2.17``:

.. code:: shell
  
  pip install pycapacity==1.2.17


v2.0.1 (09-2023)
--------------

* `Polytope <pycapacity.polytope.html#pycapacity.objects.Polytope>`_  class  - `see API docs <pycapacity.polytope.html#pycapacity.objects>`_

   * Added Minkowski sum (operator ``+``) and intersection operations (operator ``&``)
   * Added inner approximation using the Chebyshev ball (``chebyshev_ball`` method) 

* Visualisation tools - `see API docs <pycapacity.visual.html#pycapacity.visual>`_

  * ``plot_polytope`` function now has ``color`` paremeter that can be used to set ``face_color`` and ``edge_color`` and ``vertex_color`` at once - `see docs <pycapacity.visual.html#pycapacity.visual.plot_polytope>`_

* Algorithms: - `see API docs <pycapacity.algorithms.html#pycapacity.algorithms>`_

  * Added Chebyshev ball algorithm (``chebyshev_ball``) - `see docs <pycapacity.algorithms.html#pycapacity.algorithms.chebyshev_ball>`_


* Human metrics - `see API docs <pycapacity.human.html#pycapacity.human>`_

  * Added ellipsoid: metrics acceleration, velocity and force

* New module added: ``pycapacity.examples`` - `see API docs <pycapacity.examples.html#pycapacity.examples>`_

  * Contains 4 dof planar robot model - `see docs <pycapacity.examples.html#pycapacity.examples.planar_robot>`_
  * In future it will contain more toy models, for jump starting the library usage

* More examples 

  * ``examples`` folder now contains more examples on how to use the library 
      * python scripts - `see scripts <https://github.com/auctus-team/pycapacity/tree/master/examples/scripts>`_
      * jupyter notebooks - `see notebooks <https://github.com/auctus-team/pycapacity/tree/master/examples/scripts>`_
      * benchmarking scripts - `see scripts <https://github.com/auctus-team/pycapacity/tree/master/examples/scripts/benchmarking/>`_

  * More example in the docs - `see docs <examples/index.html>`_

* Journal of OpenSource software submission:

  * ``pycapacity`` paper is submitter to the Journal of OpenSource software - `see paper <https://joss.theoj.org/papers/73f155afc0dfa7730792639ac374b348>`_

* Improved the docs with more infomation about the library

  * Implemented polytope algorithms and their performance - `see docs <algorithms.html>`_
  * Implemented capacity metrics - `see docs <README.html>`_


v2.0 (05-2023)
--------------

* Polytope and ellipsoid algorithms now return `Polytope <pycapacity.polytope.html#pycapacity.objects.Polytope>`_  and `Ellipsoid <pycapacity.polytope.html#pycapacity.objects.Ellipsoid>`_  objects 

  * `Polytope <pycapacity.polytope.html#pycapacity.objects.Polytope>`_  class can be used as a standalone class to find  vertex :math:`\mathcal{V}`, halfplane :math:`\mathcal{H}` and face :math:`\mathcal{F}` representation of a polytope

* Visualisation tools improved:

  * New ``plot_polytope`` function for plotting polytopes
  * Functions ``plot_polytope_vertex`` and ``plot_polytope_faces`` now can receive a polytope object
  * Function ``plot_ellipsoid`` now can receive a ellipsoid object
  * Better management of the ``matplotlib`` figures and axes, now user can provide an ``ax``, ``plt`` or ``figure`` to the plotting functions

* Added support for robot reachable workspace approximation using convex polytopes (see `API docs <pycapacity.robot.html#pycapacity.robot.reachable_space_approximation>`_)

* Added unit testing + continuous integration
* Improved docs with sphinx

