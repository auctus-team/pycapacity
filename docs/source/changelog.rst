Changelog
=========


See `github releases <https://github.com/auctus-team/pycapacity/releases>`_ and `pypi <https://pypi.org/project/pycapacity/#history>`_ for more info. 

If wanting to use a specific version of the library, you can install it using pip, for example version ``1.2.17``:

.. code:: shell
  
  pip install pycapacity==1.2.17


v2.0 (05-2023)
--------------

* polyope and ellipsoid algorithms now return `Polytope <pycapacity.polytope.html#pycapacity.objects.Polytope>`_  and `Ellipsoid <pycapacity.polytope.html#pycapacity.objects.Ellipsoid>`_  objects 

  * `Polytope <pycapacity.polytope.html#pycapacity.objects.Polytope>`_  class can be used as a standalone class to find  vertex :math:`\mathcal{V}`, halfplane :math:`\mathcal{H}` and face :math:`\mathcal{F}` representation of a polytope

* Visualisation tools improved:

  * new ``plot_polytope`` function for plotting polytopes
  * functions ``plot_polytope_vertex`` and ``plot_polytope_faces`` now can receive a polyope object
  * function ``plot_ellipsoid`` now can receive a ellipsoid object
  * better management of the ``matplotlib`` figures and axes, now user can provide an ``ax``, ``plt`` or ``figure`` to the plotting functions

* Added support for robot reachable workspace approximation using convex polytopes (see `api docs <pycapacity.robot.html#pycapacity.robot.reachable_space_approximation>`_)

* Added unit testing + continuous integration
* improved docs with sphinx

